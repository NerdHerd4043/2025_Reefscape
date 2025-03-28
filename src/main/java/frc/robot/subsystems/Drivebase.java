// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cowlib.SwerveModule;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;
import frc.robot.Constants.PathPlannerConstants.RotationPID;
import frc.robot.Constants.PathPlannerConstants.TranslationPID;
import frc.robot.util.AutoDestinations;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightUtil;

@Logged
public class Drivebase extends SubsystemBase {
  private SwerveModule frontLeft = new SwerveModule(
      SwerveModules.frontLeft, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule frontRight = new SwerveModule(
      SwerveModules.frontRight, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule backLeft = new SwerveModule(
      SwerveModules.backLeft, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule backRight = new SwerveModule(
      SwerveModules.backRight, DriveConstants.maxVelocity, DriveConstants.maxVoltage);

  @NotLogged
  private SwerveModule[] modules = new SwerveModule[] {
      this.frontLeft,
      this.frontRight,
      this.backLeft,
      this.backRight
  };

  @NotLogged
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  @NotLogged
  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private BooleanEntry fieldOrientedEntry;

  // Limits speed of changes in direction
  @NotLogged
  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  @NotLogged
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  // Creates Sendables on the dashboard that can be interacted with to affect the
  // robot without pushing new code. These ones end up being used for scaling the
  // robot's drive speed, choosing between field and robot oriented drive, and
  // choosing the reef positions to move to.
  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
  private SendableChooser<Boolean> fieldOriented = new SendableChooser<>();
  private SendableChooser<Double> leftReefTargetPose = new SendableChooser<>();
  private SendableChooser<Double> rightReefTargetPose = new SendableChooser<>();

  // The Subscriber "subscribes" to a piece of information, allowing the
  // information to be recieved and updated. Source:
  // https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html#subscribing-to-a-topic
  @NotLogged
  private final DoubleArraySubscriber R_limelightRobotPose;
  // This double array is used later to hold information we get from the
  // subscriber. Limelight documentation (as of now) doesn't use a subscriber, but
  // our subscriber is getting the same values that the `botpose_targetspace`
  // gets, and those values are best stored in a double array. Source:
  // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
  @NotLogged
  private double[] R_limelightRobotPoseArray = new double[6];

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public double savedGyroYaw;
  private double savedAutoYaw;

  @NotLogged
  private PathConstraints constraints = new PathConstraints(
      7, // Max Velocity
      3, // Max Acceleration
      270, // Max Angular Velocity
      180 // Max Angular Acceleration
  );

  //
  //
  /** Creates a new Drivebase. */
  public Drivebase() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    // Initializes the Sendables
    this.driveSpeedChooser = new SendableChooser<>();
    this.fieldOriented = new SendableChooser<>();

    // Sets the default value of the drive speed ratio. Without this, the robot
    // won't move because it has not value to use to calculate its speed.
    this.driveSpeedChooser.setDefaultOption("Full Speed", 1.0);
    // Adds additional options for the drive speed ratio.
    this.driveSpeedChooser.addOption("Three-Quarter Speed", 0.75);
    this.driveSpeedChooser.addOption("Half Speed", 0.5);
    this.driveSpeedChooser.addOption("Quarter Speed", 0.25);
    this.driveSpeedChooser.addOption("No Speed", 0.0);

    // Sets the default drive to Field Oriented. Without a default here, the robot
    // will not enable correctly.
    this.fieldOriented.setDefaultOption("Field Oriented", true);
    // Adds Robot Oriented as an option.
    this.fieldOriented.addOption("Robot Oriented", false);

    this.leftReefTargetPose.setDefaultOption("Original", 0.0);
    this.leftReefTargetPose.addOption("0.01", 0.01);
    this.leftReefTargetPose.addOption("0.02", 0.02);
    this.leftReefTargetPose.addOption("0.03", 0.03);
    this.leftReefTargetPose.addOption("-0.01", -0.01);
    this.leftReefTargetPose.addOption("-0.02", -0.02);
    this.leftReefTargetPose.addOption("-0.03", -0.03);

    this.rightReefTargetPose.setDefaultOption("Original", 0.32);
    this.rightReefTargetPose.addOption("0.01", 0.33);
    this.rightReefTargetPose.addOption("0.02", 0.34);
    this.rightReefTargetPose.addOption("0.03", 0.35);
    this.rightReefTargetPose.addOption("-0.01", 0.31);
    this.rightReefTargetPose.addOption("-0.02", 0.30);
    this.rightReefTargetPose.addOption("-0.03", 0.29);

    // Putting Sendables on the dashboard so they can be used.
    SmartDashboard.putData("Drive Speed", this.driveSpeedChooser);
    SmartDashboard.putData("Drive Orientation", this.fieldOriented);
    SmartDashboard.putData("Left Reef Target", this.leftReefTargetPose);
    SmartDashboard.putData("Right Reef Target", this.rightReefTargetPose);

    // Putting the field on the dashboard
    SmartDashboard.putData("Field", this.field);

    this.odometry = new SwerveDriveOdometry(
        this.kinematics,
        this.getRotation2d(),
        this.getModulePositions());

    // Getting the Limelight's field position array from network tables.
    NetworkTable LLTable = inst.getTable("limelight-right");
    DoubleArrayTopic botPoseTopic = LLTable.getDoubleArrayTopic("botpose_targetspace");
    this.R_limelightRobotPose = botPoseTopic.subscribe(new double[6]);

    // Setting up auto capabilities
    RobotConfig config;

    config = Constants.DriveConstants.RobotConfigInfo.robotConfig;

    AutoBuilder.configure(
        this::getRobotPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::drive,
        new PPHolonomicDriveController(
            new PIDConstants(TranslationPID.p, TranslationPID.i, TranslationPID.d),
            new PIDConstants(RotationPID.p, RotationPID.i, RotationPID.d)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
          } else {
            return false;
          }
        },
        this);

    this.gyro.reset();

    LimelightHelpers.SetIMUMode("limelight-right", 1);
  }

  //
  //
  public double getFieldAngle() {
    return -this.gyro.getAngle();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(this.getFieldAngle());
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(this.getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public boolean getDefaultDrive() {
    return this.fieldOriented.getSelected();
  }

  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {
    if (slew) {
      speedX = this.slewRateX.calculate(speedX);
      speedY = this.slewRateY.calculate(speedY);
    }
    if (this.fieldOrientedEntry.get(this.getDefaultDrive())) {
      this.fieldOrientedDrive(speedX, speedY, rot);
    } else {
      this.robotOrientedDrive(speedX, speedY, rot);
    }
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        DriveConstants.maxVelocity);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  public double getDriverMaxVelocity() {
    return DriveConstants.maxVelocity * this.getRobotSpeedRatio();
  }

  public double getTrueMaxVelocity() {
    return DriveConstants.maxVelocity;
  }

  public double getMaxAngularVelocity() {
    return DriveConstants.maxAngularVelocity;
  }

  public Pose2d getRobotPose() {
    return this.odometry.getPoseMeters();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[this.modules.length];
    for (int i = 0; i < this.modules.length; i++) {
      positions[i] = this.modules[i].getPosition();
    }
    return positions;
  }

  // Gets the currently selected ratio for the speed, as chosen on the driver
  // station dashboard.
  public double getRobotSpeedRatio() {
    return this.driveSpeedChooser.getSelected();
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  public void resetPose(Pose2d pose2d) {
    this.odometry.resetPosition(this.getRotation2d(), this.getModulePositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Command resetGyroCommand() {
    return this.runOnce(() -> this.resetGyro());
  }

  private void saveGyroYaw() {
    this.savedGyroYaw = this.getFieldAngle();
  }

  private void saveAutoYaw() {
    this.savedAutoYaw = this.getFieldAngle();
  }

  private Pose2d endAutoPose() {
    Pose2d endPose = new Pose2d(
        this.odometry.getPoseMeters().getX(),
        this.odometry.getPoseMeters().getY(),
        new Rotation2d(this.savedGyroYaw + this.getFieldAngle() - this.savedAutoYaw));

    return endPose;
  }

  public Command resetFieldPose() {
    return this.runOnce(() -> {
      var fieldPose = LimelightUtil.getRobotFieldPose2D(
          this.gyro);
      this.resetPose(fieldPose);
    });
  }

  // Command to drive from the robot's current position (found by the Limelights)
  // to the robot's target position (calculated using the information given to
  // `AutoDestinations.getRobotFieldPose2D()`)
  public Command getAlignCommand() {
    // Initial Pose/Zero Pose
    var fieldPose = LimelightUtil.getRobotFieldPose2D(
        this.gyro);

    if (fieldPose.getX() * fieldPose.getY() == 0) {
      return Commands.runOnce(() -> System.out.println("No"));
    }
    // Final Pose
    var targetPose = AutoDestinations.destinationPose(
        LimelightUtil.getLimelightID(),
        AutoDestinations.ReefSide.LEFT,
        1.4); // FIXME: Put in dist sensor

    // Final rotation should match the final position's rotation
    var finalRotation = targetPose.getRotation();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        fieldPose,
        targetPose);

    PathConstraints constraints = this.constraints;

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, finalRotation));

    path.preventFlipping = true;

    return Commands.sequence(
        this.runOnce(() -> this.resetPose(fieldPose)),
        AutoBuilder.followPath(path));
  }

  public Command staticScoreCommand() {
    // Initial Pose/Zero Pose
    var fieldPose = LimelightUtil.getRobotFieldPose2D(
        this.gyro);

    if (fieldPose.getX() * fieldPose.getY() == 0) {
      return Commands.runOnce(() -> System.out.println("No"));
    }
    // Final Pose
    var targetPose = new Pose2d(5.63, 3, fieldPose.getRotation()); // FIXME: Put in dist sensor

    // Final rotation should match the final position's rotation
    var finalRotation = targetPose.getRotation();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        fieldPose,
        targetPose);

    PathConstraints constraints = this.constraints;

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, finalRotation));

    path.preventFlipping = true;

    return Commands.sequence(
        this.runOnce(() -> this.resetPose(fieldPose)),
        AutoBuilder.followPath(path));
  }

  // Only for testing now
  public Command autoPathTestCommand() {
    var finalRotation = Rotation2d.kCW_90deg;
    var zeroPose = new Pose2d(2, 7, new Rotation2d(0));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        zeroPose,
        new Pose2d(2.25, 7, finalRotation));

    PathConstraints constraints = this.constraints;

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
        new GoalEndState(0.0, finalRotation));

    path.preventFlipping = true;

    return Commands.sequence(
        this.runOnce(() -> this.resetPose(zeroPose)),
        AutoBuilder.followPath(path),
        Commands.runOnce(() -> System.err.println("WOWWEEEEEEE")));
  }

  public Command autoLeaveCommand() {
    var innitPose = new Pose2d(0, 0, new Rotation2d(0));
    var targetPose = new Pose2d(-2.5, 0, new Rotation2d(0));
    var finalRotation = targetPose.getRotation();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        innitPose,
        targetPose);

    PathConstraints constraints = this.constraints;

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
        new GoalEndState(0.0, finalRotation));

    path.preventFlipping = true;

    return Commands.sequence(
        this.runOnce(() -> this.resetPose(innitPose)),
        AutoBuilder.followPath(path)); // FIXME: add Field Oriented reset once it's tested
  }

  public double getRobotXPoseTargetSpace() {
    return this.R_limelightRobotPoseArray[0];
  }

  public double getLeftReefTargetPose() {
    return this.leftReefTargetPose.getSelected();
  }

  public double getRightReefTargetPose() {
    return this.rightReefTargetPose.getSelected();
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */

    // Updating odometry
    var positions = this.getModulePositions();
    this.odometry.update(this.getRotation2d(), positions);

    // Update the double array storing the field pose by getting the values from the
    // Subscriber.
    this.R_limelightRobotPoseArray = this.R_limelightRobotPose.get();

    LimelightHelpers.SetRobotOrientation("limelight-right", this.getFieldAngle(), 0, 0, 0, 0, 0);

    // Everything below is unnecessary for running the robot

    this.field.setRobotPose(this.getRobotPose()); // Shows robot pose according to odometry

    SmartDashboard.putNumber("R Target", LimelightUtil.getLimelightID());

    SmartDashboard.putNumber("Robot Pose X", LimelightUtil.getRobotPoseX()); // X Pose
    SmartDashboard.putNumber("Robot Pose Y", LimelightUtil.getRobotFieldPose2D(this.gyro).getY()); // Y Pose
    // SmartDashboard.putNumber("LL Latency", LimelightUtil.getLimelightLatency());
    // // Latency

    SmartDashboard.putNumber("Yaw", this.getFieldAngle());

    // These Smart Dashboard values are used by the CANdleSystem.java subsystem
    switch (LimelightUtil.validLimelight()) {
      case "limelight-right":
      case "limelight-left":
        SmartDashboard.putBoolean("Valid LL", true);
        break;
      default:
        SmartDashboard.putBoolean("Valid LL", false);
    }

    // LimelightUtil.smallAngleDelta();
  }
}

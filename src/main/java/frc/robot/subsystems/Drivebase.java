// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cowlib.SwerveModule;
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
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;
import frc.robot.Constants.PathPlannerConstants.RotationPID;
import frc.robot.Constants.PathPlannerConstants.TranslationPID;
import frc.robot.util.AutoDestinations;
import frc.robot.util.LimelightUtil;
import frc.robot.util.AutoDestinations.ReefSide;

public class Drivebase extends SubsystemBase {

  private SwerveModule frontLeft = new SwerveModule(
      SwerveModules.frontLeft, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule frontRight = new SwerveModule(
      SwerveModules.frontRight, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule backLeft = new SwerveModule(
      SwerveModules.backLeft, DriveConstants.maxVelocity, DriveConstants.maxVoltage);
  private SwerveModule backRight = new SwerveModule(
      SwerveModules.backRight, DriveConstants.maxVelocity, DriveConstants.maxVoltage);

  private SwerveModule[] modules = new SwerveModule[] { this.frontLeft, this.frontRight, this.backLeft,
      this.backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private BooleanEntry fieldOrientedEntry;

  // Limits speed of changes in direction
  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
  private SendableChooser<Boolean> fieldOriented = new SendableChooser<>();
  private SendableChooser<Double> inputValue = new SendableChooser<>();

  private final DoubleArraySubscriber botFieldPose;
  private double[] botFieldPoseArray = new double[6];

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final Rev2mDistanceSensor distanceSensor;

  /** Creates a new Drivebase. */
  public Drivebase() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    this.driveSpeedChooser = new SendableChooser<>();
    this.fieldOriented = new SendableChooser<>();

    this.inputValue = new SendableChooser<>();

    this.driveSpeedChooser.setDefaultOption("Full Speed", 1.0);
    this.driveSpeedChooser.addOption("Three-Quarter Speed", 0.75);
    this.driveSpeedChooser.addOption("Half Speed", 0.5);
    this.driveSpeedChooser.addOption("Quarter Speed", 0.25);
    this.driveSpeedChooser.addOption("No Speed", 0.0);

    this.fieldOriented.setDefaultOption("Field Oriented", true);
    this.fieldOriented.addOption("Robot Oriented", false);

    SmartDashboard.putData(this.driveSpeedChooser);
    SmartDashboard.putData(this.fieldOriented);
    SmartDashboard.putData("Field", this.field);

    this.odometry = new SwerveDriveOdometry(
        this.kinematics,
        this.getRotation2d(),
        this.getModulePositions());

    NetworkTable LLTable = inst.getTable("limelight-right");
    DoubleArrayTopic botPoseTopic = LLTable.getDoubleArrayTopic("botpose_orb_wpiblue");
    this.botFieldPose = botPoseTopic.subscribe(new double[6]);

    RobotConfig config;

    // try {
    // config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    // e.printStackTrace();
    config = Constants.DriveConstants.RobotConfigInfo.robotConfig;
    // }

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

    this.distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    this.distanceSensor.setAutomaticMode(true);
  }

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

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxVelocity);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  public double getMaxVelocity() {
    return DriveConstants.maxVelocity * this.getRobotSpeedRatio();
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

  public double getDistanceSensorMeters() {
    if (this.distanceSensor.isRangeValid()) {
      return this.distanceSensor.GetRange() / 100;
    } else {
      return -1;
    }
  }

  public Command getAlignCommand() {

    // Initial Pose/Zero Pose
    var fieldPose = LimelightUtil.getRobotFieldPose2D(
        this.botFieldPoseArray,
        this.gyro);
    // Final Pose and Final Roation
    var targetPose = AutoDestinations.destinationPose(
        LimelightUtil.getID("limelight-right"),
        ReefSide.LEFT,
        this.getDistanceSensorMeters());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        fieldPose,
        targetPose);

    PathConstraints constraints = new PathConstraints(
        8, // Max Velocity
        3, // Max Acceleration
        270, // Max Angular Velocity
        180 // Max Angular Acceleration
    );

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, targetPose.getRotation()));

    path.preventFlipping = true;

    return Commands.sequence(
        this.runOnce(() -> this.resetPose(fieldPose)),
        AutoBuilder.followPath(path));
  }

  public Command autoPathTestCommand() {
    var finalRotation = Rotation2d.kZero;
    var zeroPose = new Pose2d(2, 7, new Rotation2d(0));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        zeroPose,
        new Pose2d(4, 7, finalRotation));

    PathConstraints constraints = new PathConstraints(
        8, // Max Velocity
        3, // Max Acceleration
        270, // Max Angular Velocity
        180 // Max Angular Acceleration
    );

    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
        new GoalEndState(0.0, finalRotation));

    path.preventFlipping = true;

    return Commands.sequence(this.runOnce(() -> this.resetPose(zeroPose)), AutoBuilder.followPath(path));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var positions = this.getModulePositions();
    this.odometry.update(this.getRotation2d(), positions);

    if (this.distanceSensor.isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", this.getDistanceSensorMeters());
    }

    // Everything below is unnecessary for running the robot

    this.botFieldPoseArray = this.botFieldPose.get();

    this.field.setRobotPose(this.getRobotPose());

    SmartDashboard.putNumber("Speed Ratio", this.getRobotSpeedRatio());

    SmartDashboard.putNumber("R Target", LimelightUtil.getID("limelight-right"));

    SmartDashboard.putNumber("Field Pose X", this.botFieldPoseArray[0]); // Field Y Pose
    SmartDashboard.putNumber("Field Pose Y", this.botFieldPoseArray[1]); // Field Y Pose
    SmartDashboard.putNumber("LL Latency", this.botFieldPoseArray[5]); // Latency
  }
}

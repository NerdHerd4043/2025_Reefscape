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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;

public class Drivebase extends SubsystemBase {

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

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

  // Limits speed of changes in direction
  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
  private SendableChooser<Boolean> fieldOriented = new SendableChooser<>();

  private BooleanEntry fieldOrientedEntry;

  /** Creates a new Drivebase. */
  public Drivebase() {
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    this.driveSpeedChooser = new SendableChooser<>();
    this.fieldOriented = new SendableChooser<>();

    this.driveSpeedChooser.setDefaultOption("Full Speed", 1.0);
    this.driveSpeedChooser.addOption("Three-Quarter Speed", 0.75);
    this.driveSpeedChooser.addOption("Half Speed", 0.5);
    this.driveSpeedChooser.addOption("Quarter Speed", 0.25);
    this.driveSpeedChooser.addOption("No Speed", 0.0);

    this.fieldOriented.setDefaultOption("Field Oriented", true);
    this.fieldOriented.addOption("Robot Oriented", false);

    SmartDashboard.putData(this.driveSpeedChooser);
    SmartDashboard.putData(this.fieldOriented);

    this.odometry = new SwerveDriveOdometry(this.kinematics, this.gyro.getRotation2d(), this.getModulePositions());
  }

  public double getFieldAngle() {
    return -this.gyro.getYaw();
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

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxVelocity * this.getRobotSpeedRatio());

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  public double getMaxVelocity() {
    return DriveConstants.maxVelocity;
  }

  public double getMaxAngularVelocity() {
    return DriveConstants.maxAngularVelocity;
  }

  public Pose2d getRobotPose() {
    return this.odometry.getPoseMeters();
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

  public Command resetGyroCommand() {
    return this.run(this::resetGyro);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var positions = this.getModulePositions();

    this.odometry.update(this.gyro.getRotation2d(), positions);
    var pose = this.getRobotPose();

    // Everything below is unnecessary for running the robot
    var translation = pose.getTranslation();
    var x = translation.getX();
    var y = translation.getY();
    var rotation = pose.getRotation().getDegrees();
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("rot", rotation);
    this.field.setRobotPose(this.getRobotPose());

    SmartDashboard.putNumber("Speed Ratio", this.getRobotSpeedRatio());
  }
}

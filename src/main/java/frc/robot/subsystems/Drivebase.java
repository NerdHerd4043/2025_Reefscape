// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import cowlib.SwerveModule;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private final double DRIVE_REDUCTION = 1.0 / 6.12; // Where to find drive reduction (for swerve):
                                                     // (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
  private final double NEO_FREE_SPEED = 5820.0 / 60.0; // Get from website, divide by 60 since the number that the
                                                       // website gives is in rotations per minute and we want
                                                       // rotations per second.
  private final double WHEEL_DIAMETER = 0.1016; // Measured by hand
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI; // Max velocity =
                                                                                                   // Fastest the
                                                                                                   // motor can spin
                                                                                                   // (free speed)
                                                                                                   // times the gear
                                                                                                   // ratio times
                                                                                                   // the
                                                                                                   // circumference
                                                                                                   // of
                                                                                                   // the wheel
                                                                                                   // (diameter
                                                                                                   // (twice
                                                                                                   // the radius)
                                                                                                   // times pi).
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (ModuleLocations.robotRaduius); // Max velocity applied
                                                                                             // tangentially (in a
                                                                                             // direction that causes
                                                                                             // the robot to spin)
                                                                                             // divided by the radius
                                                                                             // of the robot (center
                                                                                             // to wheel) is equal to
                                                                                             // the max angular
                                                                                             // velocity.
                                                                                             // Rearrangement of basic
                                                                                             // physics formula.

  private final double MAX_VOLTAGE = 12; // This is the battery voltage.

  private AHRS gyro;

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft, MAX_VELOCITY, MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight, MAX_VELOCITY, MAX_VOLTAGE);

  private SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate); // Limits speed of changes in
                                                                                    // direction
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();

  private BooleanEntry fieldOrientedEntry;

  /** Creates a new Drivebase. */
  public Drivebase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

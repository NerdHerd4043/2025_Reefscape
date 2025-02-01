// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cowlib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
  public static final class DriveConstants {
    public static final double deadband = 0.09;
    public static final int currentLimit = 40;
    public static final double slewRate = 20; // lower number for higher center of mass
    public static final int temp = 21;
    public static final double DRIVE_REDUCTION = 1.0 / 6.75;
    public static final double NEO_FREE_SPEED = 5820.0 / 60.0;
    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY
        / (ModuleLocations.moduleLocationLength / Math.sqrt(2.0));

    public static final class SwervePID {
      public static final double p = 0.12;
      public static final double i = 0;
      public static final double d = 0.0015;
    }

    public static final class SwerveModules {
      public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(1, 11, 21, false);
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(2, 12, 22, true);
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(3, 13, 23, false);
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(4, 14, 24, true);
    }

    public static final class ModuleLocations {
      public static final double moduleLocationLength = Units.inchesToMeters(12.00);
      public static final double moduleLocationWidth = Units.inchesToMeters(12.25);
      public static final double robotRaduius = Math
          .sqrt(Math.pow(moduleLocationLength, 2) + Math.pow(moduleLocationWidth, 2));
      public static final Translation2d frontLeft = new Translation2d(moduleLocationLength, moduleLocationLength);
      public static final Translation2d frontRight = new Translation2d(moduleLocationLength, -moduleLocationLength);
      public static final Translation2d backLeft = new Translation2d(-moduleLocationLength, moduleLocationLength);
      public static final Translation2d backRight = new Translation2d(-moduleLocationLength, -moduleLocationLength);
    }
  }

  public static final class CoralIntake {
    public static final int motorId = 0; // FIXME: Need a motor id

    public static final double intakeSpeed = 0.5;
  }

  public static final class CoralWrist {
    public static final int motorId = 0; // FIXME: Need a motor id
    public static final int encoderID = 0; // FIXME: Need ID

    public static final int currentLimit = 25;

    // FIXME: Find Constraints
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        0, 0);

    // FIXME: Tune
    public static final class PIDValues {
      public static final double p = 0;
      public static final double i = 0;
      public static final double d = 0;
    }

    // FIXME: Tune
    public static final class FeedForwardValues {
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
    }

    // FIXME: Find limits
    public static final class WristPositions {
      public static final double lower = 0;
      public static final double upper = 1;
      public static final double stationPos = 1;
      public static final double branchPos = 0.5;
    }
  }

  public static final class Elevator {
    public static final int leftMotorId = 0; // FIXME: Need a motor id
    public static final int rightMotorId = 0; // FIXME: Need a motor id
  }
}

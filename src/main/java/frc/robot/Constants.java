// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import cowlib.SwerveModuleConfig;
import cowlib.Util;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
  public static final class DriveConstants {
    // Radius (maybe) of the area that the drivestick input has no effect
    public static final double deadband = 0.09;
    public static final double autoCancelThreshold = 0.35;

    public static final int currentLimit = 40;

    // Lower number for higher center of mass
    public static final double slewRate = 20;

    // Where to find drive reduction (for swerve):
    // (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
    // L1: 1.0 / 8.14 ; L2: 1.0 / 6.75 ; L3: 1.0 / 6.12
    // ALSO CHANGE IN COWLIB.SWERVEMODULE!!!!!!!!!!!!!!!!!!!!!! <----------
    public static final double driveGearing = 6.12;
    public static final double driveReduction = 1.0 / driveGearing;

    // Get from website, divide by 60 since the number that the
    // website gives is in rotations per minute and we want
    // rotations per second.
    public static final double neoFreeSpeed = 5820.0 / 60.0;

    // Measured by hand, write in meters
    public static final double wheelDiameter = 0.1016;

    // Max velocity = Fastest the motor can spin (free speed) times the gear ratio
    // times the circumference of the wheel (diameter (twice the radius) times pi).
    public static final double maxVelocity = neoFreeSpeed * driveReduction * wheelDiameter * Math.PI;

    // Max velocity applied tangentially (in a direction that causes the robot to
    // spin) divided by the radius of the robot (center to wheel) is equal to the
    // max angular velocity. Rearrangement of basic physics formula.
    public static final double maxAngularVelocity = maxVelocity / (ModuleLocations.robotRaduius);

    // This is the nominal battery voltage.
    public static final double maxVoltage = 12;

    // The PID loop that runs with our swerve. Don't touch unless necessary.
    public static final class SwervePID {
      public static final double p = 0.12;
      public static final double i = 0;
      public static final double d = 0.0015;
    }

    // Creates SwerveModules for use in Drivebase. Sets IDs for the motors and
    // encoder of each module, as well as direction of drive.
    public static final class SwerveModules {
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(1, 11, 21, true);
      public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(2, 12, 22, true);
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(3, 13, 23, true);
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(4, 14, 24, true);
    }

    public static final class ModuleLocations {
      // Measurement from center of one wheel to center of the other divided by 2. Ex:
      // this robot was 24 inches in length between center of wheels and 24.5 inches
      // in width.
      public static final double moduleLocationLength = Units.inchesToMeters(12.00);
      public static final double moduleLocationWidth = Units.inchesToMeters(12.25);

      // The robot's radius is the hypotenuse of the triangle with legs made by the
      // length and width defined above. Math is done automatically when the length
      // and width are changed.
      // Formula: square root((Length)^2 + (Width)^2) = Radius
      public static final double robotRaduius = Math
          .sqrt(Math.pow(moduleLocationLength, 2) + Math.pow(moduleLocationWidth, 2));

      // Locations of the wheels relative to the center of the robot.
      public static final Translation2d frontLeft = new Translation2d(moduleLocationLength, moduleLocationLength);
      public static final Translation2d frontRight = new Translation2d(moduleLocationLength, -moduleLocationLength);
      public static final Translation2d backLeft = new Translation2d(-moduleLocationLength, moduleLocationLength);
      public static final Translation2d backRight = new Translation2d(-moduleLocationLength, -moduleLocationLength);
    }

    // This replaces the config in the PathPlanner app GUI
    public static final class RobotConfigInfo {
      public static final ModuleConfig moduleConfig = new ModuleConfig(
          wheelDiameter,
          10,
          1,
          DCMotor.getNEO(1),
          driveGearing,
          DriveConstants.currentLimit,
          1);

      public static final RobotConfig robotConfig = new RobotConfig(
          50.35,
          // Equation: Mass * radius^2
          25.000,
          moduleConfig,
          ModuleLocations.frontLeft,
          ModuleLocations.frontRight,
          ModuleLocations.backLeft,
          ModuleLocations.backRight);
    }
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 1.55;
      public static final double i = 0.1;
      public static final double d = 1;
    }

    public static final class RotationPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static final class AlgaeIntake {
    public static final int motorID = 50;
    public static final int currentLimit = 30;
    public static final double intakeSpeed = 0;
  }

  public static final class AlgaeWrist {
    public static final int motorID = 51;
    public static final int currentLimit = 30;

    public static final TrapezoidProfile.Constraints constraintsA = new TrapezoidProfile.Constraints(
        0, 0);

    public static final class PIDValuesA {
      public static final double p = 0;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class FeedForwardValuesA {
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
    }

    public static final class WristPositionsA {
      public static final double lower = 0;
      public static final double upper = 1;
    }
  }

  public static final class CoralIntake {
    public static final int motorId = 30;
    public static final double intakeSpeed = 0.6;
    public static final double outtakeSpeed = 0.5;
    public static final int currentLimit = 40;

    public static final double distSensorLow = 29;
    public static final double distSensorHigh = 409;
    public static final double distSensorHighNoCoral = 540;

    public static final double offsetLowMeters = 0;
    public static final double offsetHighMeters = 0.3556;

    public static double mapCoral(double sensor) {
      return Util.mapDouble(sensor,
          Constants.CoralIntake.distSensorLow,
          Constants.CoralIntake.distSensorHigh,
          Constants.CoralIntake.offsetLowMeters,
          Constants.CoralIntake.offsetHighMeters);
    }

    public static final int distSensorID = 33;
  }

  public static final class CoralWrist {
    public static final int motorId = 31;
    public static final int encoderID = 32;
    public static final int currentLimit = 30;

    // Limits the CoralWrist PID Controller
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        6, 10);

    // Tuned manually. Practice here:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    // FIXME: Fine-tune
    public static final class PIDValuesC {
      public static final double p = 10;
      public static final double i = 0;
      public static final double d = 0;
    }

    // Found manually. Put the encoder value on the driver station dashboard and
    // move the encoder to the desired position, then record the value.
    public static final class WristPositionsC {
      public static final double lower = 0;
      public static final double upper = 1.20;
      public static final double stationPos = 1.20;
      public static final double L2BranchPos = 0.43;
      public static final double highBranchesPos = 0;
    }
  }

  // todo: give climber actual values
  public static final class Climber {
    public static final int rightMotorId = 60;
    public static final int leftMotorId = 62;
    public static final int encoderID = 61;
    public static final int currentLimit = 40;

    public static final double maxClimberRotation = 1.23;

    // Limits the Climber PID Controller
    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        3.2, 2.5);

    // Tuned manually. Practice here:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    public static final class PIDValuesC {
      public static final double p = 2.8;
      public static final double i = 1.4;
      public static final double d = 0;
    }

    // Found manually. Put the encoder value on the driver station dashboard and
    // move the encoder to the desired position, then record the value.
    public static final class ClimberPositionsC {
      public static final double lower = 0;
      public static final double upper = 1.23;
      public static final double upPos = 0.43;
      public static final double downPos = 0;
    }
  }

  public static final class Elevator {
    public static final int currentLimit = 40;

    public static final TrapezoidProfile.Constraints constraints = // I just lost the game
        new TrapezoidProfile.Constraints(90, 190);

    public static final int leftMotorId = 41;
    public static final int rightMotorId = 40;

    // This is an encoder value found by manually raising the elevator to max height
    // after it was zeroed at the bottom and while the encoder value was put on the
    // dashboard.
    public static final double maxElevatorHeight = 134.49;

    public static final double[] elevatorHeights = {
        0.0, // Colapse
        4, // Coral station height (Ours was about 36.5 in) - 3.125, 4.25+ ; 3.125, 4-
        41, // L2 (we don't use this, it's a placeholder)
        63, // L3
        maxElevatorHeight * 0.99 // L4
    };

    // Tuned manually. Practice here (different than the arm):
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
    public static final class PIDValuesE {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    // Tuned manually. Practice here (different than the arm):
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
    public static final class FeedForwardValues {
      public static final double kS = 0;
      public static final double kG = 0.33;
      public static final double kV = 0;
    }
  }

  public static final class CANdleConstants {
    public static final int CANdleID = 38;
    public static final int ledCount = 80;
  }

}

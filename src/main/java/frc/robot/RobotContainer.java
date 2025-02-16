// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralWrist;

public class RobotContainer {

  private static final CommandXboxController driveStick = new CommandXboxController(0);

  private static final Drivebase drivebase = new Drivebase();

  private static final Elevator elevator = new Elevator();
  private static final CoralWrist coralWrist = new CoralWrist();
  private static final CoralIntake coralIntake = new CoralIntake();

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> getScaledXY(),
            () -> scaleRotationAxis(driveStick.getRightX())));

    configureBindings();
  }

  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double[] getXY() {
    double[] xy = new double[2];
    xy[1] = deadband(driveStick.getLeftX(), DriveConstants.deadband);
    xy[0] = deadband(driveStick.getLeftY(), DriveConstants.deadband);
    return xy;
  }

  private double getElevatorSpeedRatio() {
    if (elevator.isElevatorExtended()) {
      return 0.5;
    } else {
      return 1;
    }
  }

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Converting to Polar coordinates (uses coordinates (r, theta) where `r` is
    // magnitude and `theta` is the angle relative to 0. Usually 0 is in the
    // positive direction of a cartesian graph's x axis, and increases positively
    // with counterclockwise rotation).
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[0], xy[1]);

    // Square radius and scale by max velocity. This allows for slower speed when
    // the drivestick is closer to the center without limiting the max speed because
    // 1 is the max output of the drivestick and 1 * 1 = 1.
    r = r * r * drivebase.getMaxVelocity();

    // Convert to Cartesian coordinates (uses coordinates (x,y)) by getting the `x`
    // and `y` legs of the right triangle where `r` is the hypotenuse and `x` and
    // `y` are the legs. Learn trigonometry *shrug*. Also multiplies by 0.5 if the
    // elevator is in use so the robot has smaller chances of tipping.
    xy[1] = r * Math.cos(theta) * getElevatorSpeedRatio();
    xy[0] = r * Math.sin(theta) * getElevatorSpeedRatio();

    return xy;
  }

  private double scaleRotationAxis(double input) {
    return deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngularVelocity() * 0.6;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  private void configureBindings() {
    driveStick.rightBumper().whileTrue(coralIntake.intakeCommand());
    driveStick.leftBumper().whileTrue(coralIntake.outtakeCommand());

    driveStick.povUp().onTrue(this.coralWrist.getStationCommand());
    driveStick.povDown().onTrue(this.coralWrist.getBranchCommand());

    driveStick.y().onTrue(elevator.getExtendCommand(3));
    driveStick.x().onTrue(elevator.getExtendCommand(2));
    driveStick.a().onTrue(elevator.getExtendCommand(1));
    driveStick.b().whileTrue(elevator.getCollapseCommand());

    // Reset gyro button
    driveStick.povUpLeft().onTrue(drivebase.getResetGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

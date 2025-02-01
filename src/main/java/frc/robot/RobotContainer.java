// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CoralWrist;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Elevator;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.coral.CoralIntake;

public class RobotContainer {

  private static CommandXboxController driveStick = new CommandXboxController(0);

  final Drivebase drivebase = new Drivebase();

  final Elevator elevator = new Elevator();

  final CoralWrist coralWrist = new CoralWrist();

  private static CoralIntake coralIntake = new CoralIntake();

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

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Convert to Polar coordinates
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[0], xy[1]);

    // Square radius and scale by max velocity
    r = r * r * drivebase.getMaxVelocity();

    // Convert to Cartesian coordinates
    xy[1] = r * Math.cos(theta);
    xy[0] = r * Math.sin(theta);

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

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

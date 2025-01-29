// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coral.CoralIntake;

public class RobotContainer {
  private static CoralIntake coralIntake = new CoralIntake();

  private static CommandXboxController driveStick = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveStick.rightBumper().whileTrue(coralIntake.intakeCommand());
    driveStick.leftBumper().whileTrue(coralIntake.outtakeCommand());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

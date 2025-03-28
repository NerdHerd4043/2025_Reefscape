// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coral.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleOnIntake extends Command {

  private CoralIntake coralIntake;
  private CommandXboxController driveStick;

  private boolean timeSet;
  private double time;

  /** Creates a new RumbleOnIntake. */
  public RumbleOnIntake(CoralIntake coralIntake, CommandXboxController driveStick) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.coralIntake = coralIntake;
    this.driveStick = driveStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timeSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.coralIntake.pieceAquired()) {
      this.driveStick.setRumble(RumbleType.kBothRumble, 0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveStick.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

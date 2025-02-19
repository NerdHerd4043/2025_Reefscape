// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.algae.AlgaeWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroAlgaeWrist extends Command {
  private final AlgaeWrist algaeWrist;

  private double previousPosition;
  private boolean firstRun = true;

  /** Creates a new ZeroAlgaeWrist. */
  public ZeroAlgaeWrist(AlgaeWrist algaeWrist) {
    this.algaeWrist = algaeWrist;

    addRequirements(this.algaeWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.previousPosition = algaeWrist.encoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.algaeWrist.creepUpward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.algaeWrist.stop();
    this.algaeWrist.resetPosition();
    this.algaeWrist.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (firstRun) {
      this.firstRun = false;
      return false;
    }

    double positionDelta = Math.abs(this.previousPosition - this.algaeWrist.encoderPosition());

    if (positionDelta < Constants.AlgaeWrist.zeroThreshold) {
      return true;
    }

    this.previousPosition = this.algaeWrist.encoderPosition();

    return false;
  }
}

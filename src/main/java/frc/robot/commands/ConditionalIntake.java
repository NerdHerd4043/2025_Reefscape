// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ConditionalIntake extends Command {
  CoralIntake coralIntake;

  double time;
  double deltaTime;

  /** Creates a new ConditionalIntake. */
  public ConditionalIntake(CoralIntake coralIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralIntake = coralIntake;

    this.addRequirements(this.coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.coralIntake.runIntake(Constants.CoralIntake.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.coralIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(this.coralIntake.pieceAquired());
    return this.coralIntake.pieceAquired();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

import cowlib.Util;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlignCommand extends Command {

  private final Drivebase drivebase;

  private boolean finished = false;

  /** Creates a new ReefAlignCommand. */
  public ReefAlignCommand(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    this.addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotPoseX = this.drivebase.getRobotXPoseTargetSpace();
    double targetPoseX = 0.03; // FIXME: TUNE BEFORE FULL USE

    double maxOffset = 1; // FIXME: Find range
    double deadband = 0; // FIXME: Tune

    double deltaX = MathUtil.clamp(robotPoseX - targetPoseX, -maxOffset, maxOffset);
    double speedX = Math.copySign(Util.mapDouble(deltaX, 0, maxOffset, 0, this.drivebase.getTrueMaxVelocity() * 0.65),
        deltaX);

    // In robotOrientedDrive: Positive x moves the robot forward, positive y moves
    // the robot left
    if (Math.abs(deltaX) > deadband) {
      this.drivebase.robotOrientedDrive(0, speedX, 0);
    } else {
      // We may need extra movement here to cancel our momentum, but we can also
      // decrease the speed by decreasing the max velocity and see if that works.
      this.finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.robotOrientedDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}

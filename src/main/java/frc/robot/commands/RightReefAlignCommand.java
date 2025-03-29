// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.LimelightUtil;
import cowlib.Util;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RightReefAlignCommand extends Command {

  private final Drivebase drivebase;

  private double time;
  private boolean timeSet;
  private boolean finished = false;

  /** Creates a new ReefAlignCommand. */
  public RightReefAlignCommand(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    this.addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timeSet = false;
    this.finished = false;
  }

  public boolean pieceAquired() {
    return SmartDashboard.getNumber("Distance Sensor",
        Constants.CoralIntake.distSensorLow) < Constants.CoralIntake.distSensorHighNoCoral - 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distSensorOffset = Util.mapDouble(
        SmartDashboard.getNumber("Distance Sensor", Constants.CoralIntake.distSensorLow),
        Constants.CoralIntake.distSensorLow,
        Constants.CoralIntake.distSensorHigh,
        0,
        0.3556);

    SmartDashboard.putNumber("Left Limelight Last Change", LimelightUtil.L_limelightRobotPose.getLastChange());
    SmartDashboard.putNumber("Right Limelight Last Change", LimelightUtil.R_limelightRobotPose.getLastChange());
    double robotPoseX = LimelightUtil.getRobotPoseX();
    double targetPoseX = this.drivebase.getRightReefTargetPose();

    double maxOffset = 1;
    double deadband = 0.015;

    double deltaX = MathUtil.clamp(robotPoseX - targetPoseX + distSensorOffset, -maxOffset, maxOffset);
    double speedX = Math.copySign(Util.mapDouble(deltaX, 0, maxOffset, 0, this.drivebase.getTrueMaxVelocity() * 0.7),
        deltaX);

    // System.out.println("Delta X" + deltaX);
    // System.out.println("Speed X" + speedX);

    // In robotOrientedDrive: Positive x moves the robot forward, positive y moves
    // the robot left
    if (Math.abs(deltaX) > deadband) {
      this.timeSet = false;
      this.drivebase.robotOrientedDrive(0, speedX, 0);
      // This Smart Dashboard value is used by the CANdleSystem.java subsystem
      SmartDashboard.putBoolean("Aligning", true);
      System.err.println(deltaX);
    } else {
      // We may need extra movement here to cancel our momentum, but we can also
      // decrease the speed by decreasing the max velocity and see if that works.
      this.drivebase.robotOrientedDrive(0, 0, 0);

      if (!this.timeSet) {
        this.time = Timer.getFPGATimestamp();
        this.timeSet = true;
        System.out.println("Time Set");
      }

      double deltaTime = Timer.getFPGATimestamp() - this.time;
      System.out.println(deltaTime);

      if (deltaTime > 1 && deltaX < deadband) {
        System.err.println("Finished!!!!!!!!!!!!");
        this.finished = true;
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivebase.robotOrientedDrive(0, 0, 0);

    // These Smart Dashboard values are used by the CANdleSystem.java subsystem
    SmartDashboard.putBoolean("Aligning", false);
    SmartDashboard.putBoolean("Aligned", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished || !pieceAquired();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import cowlib.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlignmentSystem;
import frc.robot.subsystems.Drivebase;
import frc.robot.util.AutoDestinations.ReefSide;
import frc.robot.util.LimelightUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlignCommand extends Command {
  private final Drivebase drivebase;
  private ReefSide side;

  private double time;
  private boolean timeSet;
  private boolean finished = false;

  /** Creates a new ReefAlignCommand. */
  public ReefAlignCommand(Drivebase drivebase, ReefSide side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    this.side = side;

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

    System.out.println("Align Exe");
    double robotPoseX = LimelightUtil.getRobotPoseX();
    double deltaX = AlignmentSystem.getDeltaX(side);
    double maxOffset = AlignmentSystem.getMaxOffset();
    double speedX = Math.copySign(
        Util.mapDouble(
            deltaX, 0,
            maxOffset, 0,
            this.drivebase.getTrueMaxVelocity() * 0.7),
        deltaX);

    System.out.println(AlignmentSystem.getSideAligned(side));
    System.out.print(robotPoseX);
    System.out.print(" ");
    System.out.print(deltaX);
    System.out.print(" ");
    System.out.println(speedX);

    // In robotOrientedDrive: Positive x moves the robot forward, positive y moves
    // the robot left
    if (robotPoseX != 0 && !AlignmentSystem.getSideAligned(side)) {
      this.timeSet = false;
      this.drivebase.robotOrientedDrive(0, speedX, 0);
      // This Smart Dashboard value is used by the CANdleSystem.java subsystem
      SmartDashboard.putBoolean("Aligning", true);
    } else {
      // We may need extra movement here to cancel our momentum, but we can also
      // decrease the speed by decreasing the max velocity and see if that works.
      this.drivebase.robotOrientedDrive(0, 0, 0);

      if (!this.timeSet) {
        this.time = Timer.getFPGATimestamp();
        this.timeSet = true;
      }

      double deltaTime = Timer.getFPGATimestamp() - this.time;

      if (deltaTime > 1 && robotPoseX != 0) {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished || !pieceAquired();
  }
}

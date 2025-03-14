// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algae.AlgaeIntake;
import frc.robot.subsystems.algae.AlgaeWrist;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralWrist;
import frc.robot.subsystems.Climber;

import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.playingwithfusion.TimeOfFlight;
import com.pathplanner.lib.auto.NamedCommands;

import cowlib.Util;

public class RobotContainer {

  private static final CommandXboxController driveStick = new CommandXboxController(0);

  private static final Drivebase drivebase = new Drivebase();

  private static final Elevator elevator = new Elevator();

  private static final CoralWrist coralWrist = new CoralWrist();
  private static final CoralIntake coralIntake = new CoralIntake();

  private static final Climber climber = new Climber();

  // private static final AlgaeWrist algaeWrist = new AlgaeWrist();
  // private static final AlgaeIntake algaeIntake = new AlgaeIntake();

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            this::getScaledXY,
            () -> scaleRotationAxis(driveStick.getRightX())));

    this.configureBindings();

    climber.setDefaultCommand(climber.run(() -> climber.setSpeed(
        driveStick.getLeftTriggerAxis() - driveStick.getRightTriggerAxis())));

    // Limits which IDs of April Tags the Limelight is able to target.
    int[] validIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", validIDs);

    NamedCommands.registerCommand("L2 Score",
        Commands.race(
            Commands.parallel(
                coralWrist.L2BranchCommand(),
                Commands.waitSeconds(2))));

    NamedCommands.registerCommand("L3 Score",
        Commands.race(
            Commands.sequence(
                Commands.parallel(
                    elevator.extendCommand(3),
                    coralWrist.highBranchesCommand()),
                Commands.waitSeconds(4),
                coralIntake.outtakeCommand().withTimeout(2),
                elevator.collapseCommand(),
                coralWrist.stationCommand())));

    NamedCommands.registerCommand("L4 Score",
        Commands.race(
            Commands.sequence(
                Commands.parallel(
                    elevator.extendCommand(4),
                    coralWrist.highBranchesCommand()),
                Commands.waitSeconds(4),
                coralIntake.outtakeCommand().withTimeout(2),
                elevator.collapseCommand(),
                coralWrist.stationCommand())));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // CameraServer.startAutomaticCapture(0);
  }

  // Used to create an area around the center of the joystick where the input is
  // 0, so as to avoid stick drift.
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double[] getScaledXY() {
    double[] xy = new double[2];

    xy[0] = deadband(-driveStick.getLeftY(), DriveConstants.deadband);
    xy[1] = deadband(-driveStick.getLeftX(), DriveConstants.deadband);

    Util.square2DVector(xy);

    // Scales the max drive speed when the elevator is enabled.
    var scaling = drivebase.getDriverMaxVelocity() * this.getElevatorSpeedRatio();
    xy[0] *= scaling;
    xy[1] *= scaling;

    return xy;
  }

  // Used to scale the drive speed when the elevator is enabled.
  private double getElevatorSpeedRatio() {
    if (elevator.encoderPosition() > 70) {
      return 0.5;
    } else {
      return 1;
    }
  }

  private double scaleRotationAxis(double input) {
    return this.deadband(this.squared(input), DriveConstants.deadband) * drivebase.getMaxAngularVelocity() * -0.6;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  public void resetGyro() {
    drivebase.resetGyroCommand();
  }

  public Command resetCoralPID() {
    return coralWrist.resetPIDCommand();
  }

  private void configureBindings() {
    /* Intake/Output buttons */
    // Intake
    driveStick.leftBumper().whileTrue(
        Commands.parallel(
            elevator.extendCommand(1),
            coralIntake.intakeCommand(),
            climber.stationCommand(),
            coralWrist.stationCommand()));
    // Output
    driveStick.rightBumper().whileTrue(coralIntake.outtakeCommand());

    /* Coral wrist angle buttons */
    driveStick.povRight().onTrue(coralWrist.L2BranchCommand()); // Wrist straightish

    /* Elevator height and coral wrist angle (at the same time) buttons */
    // Coral station position
    driveStick.b().whileTrue(
        Commands.parallel(
            elevator.collapseCommand(),
            climber.stationCommand(),
            coralWrist.stationCommand()));

    // L2
    driveStick.a().onTrue(Commands.parallel(elevator.collapseCommand(), coralWrist.L2BranchCommand()));
    // L3
    driveStick.x().onTrue(
        Commands.parallel(
            elevator.extendCommand(3),
            coralWrist.highBranchesCommand()));
    // L4
    driveStick.y().onTrue(
        Commands.parallel(
            elevator.extendCommand(4),
            coralWrist.highBranchesCommand()));

    /* Auto testing buttons */
    driveStick.start().whileTrue(new ReefAlignCommand(drivebase));

    driveStick.rightStick().whileTrue(coralIntake.intakeCommand());

    /* Reset gyro button */
    driveStick.povLeft().onTrue(drivebase.resetGyroCommand());

    driveStick.back().whileTrue(elevator.coastModeCommand());
  }

  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();

    // return Commands.sequence(
    // drivebase.resetFieldPose(),
    // this.autoChooser.getSelected(),
    // Commands.race(drivebase.run(() -> drivebase.defaultDrive(0, 0, 0, false)),
    // Commands.sequence(Commands.parallel(elevator.extendCommand(4),
    // coralWrist.highBranchesCommand()),
    // Commands.waitSeconds(3),
    // coralIntake.outtakeCommand().withTimeout(2),
    // elevator.collapseCommand(),
    // coralWrist.stationCommand())));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ConditionalIntake;
import frc.robot.commands.Drive;
import frc.robot.commands.NoDrive;
import frc.robot.commands.RightReefAlignCommand;
import frc.robot.commands.RumbleOnIntake;
import frc.robot.commands.LeftReefAlignCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.algae.AlgaeIntake;
import frc.robot.subsystems.algae.AlgaeWrist;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralWrist;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Climber;

import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import cowlib.Util;

@Logged
public class RobotContainer {
  private final CommandXboxController driveStick = new CommandXboxController(0);

  private final Drivebase drivebase = new Drivebase();
  private final Elevator elevator = new Elevator();
  private final CoralWrist coralWrist = new CoralWrist();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Climber climber = new Climber();

  private final CANdleSystem CANdle = new CANdleSystem();

  private SendableChooser<Command> autoChooser;

  private Command fixCoral = Commands.sequence(
      coralIntake.outtakeCommand().withTimeout(0.1),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.1),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.1),
      coralIntake.intakeCommand().withTimeout(0.25));

  public RobotContainer() {
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            this::getScaledXY,
            () -> scaleRotationAxis(driveStick.getRightX())));

    climber.setDefaultCommand(climber.run(() -> climber.setSpeed(
        driveStick.getLeftTriggerAxis() - driveStick.getRightTriggerAxis())));

    // Limits which IDs of April Tags the Limelight is able to target.
    int[] validIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", validIDs);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", validIDs);

    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.setPipelineIndex("limelight-right", 0);

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
                elevator.collapseCommand().ignoringDisable(true),
                coralWrist.stationCommand())));

    NamedCommands.registerCommand("L4 Score",
        Commands.race(
            Commands.sequence(
                Commands.parallel(
                    elevator.extendCommand(4),
                    coralWrist.highBranchesCommand()),
                Commands.waitSeconds(2),
                coralIntake.outtakeCommand().withTimeout(1),
                elevator.collapseCommand(),
                coralWrist.stationCommand())));

    NamedCommands.registerCommand("Reef Align", new LeftReefAlignCommand(drivebase));

    NamedCommands.registerCommand("Conditional Intake", new ConditionalIntake(coralIntake));

    NamedCommands.registerCommand("No Drive", new NoDrive(drivebase));

    NamedCommands.registerCommand("Elevator L4", Commands.parallel(
        elevator.extendCommand(4),
        coralWrist.highBranchesCommand()));

    NamedCommands.registerCommand("Score", Commands.sequence(
        Commands.waitSeconds(1.5),
        coralIntake.outtakeCommand().withTimeout(1),
        elevator.collapseCommand(),
        coralWrist.stationCommand()));

    NamedCommands.registerCommand("Straighten Coral", this.fixCoral);

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    CameraServer.startAutomaticCapture(0);

    this.configureBindings();
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
            coralWrist.stationCommand(),
            new RumbleOnIntake(coralIntake, driveStick)));
    // Output
    driveStick.rightBumper().whileTrue(coralIntake.outtakeCommand());

    driveStick.povDown().onTrue(this.fixCoral);

    /* Coral wrist angle buttons */
    // driveStick.povRight().onTrue(coralWrist.L2BranchCommand()); // Wrist
    // straightish

    /* Elevator height and coral wrist angle (at the same time) buttons */
    // Coral station position
    driveStick.b().whileTrue(
        Commands.parallel(
            elevator.collapseCommand(),
            climber.stationCommand(),
            coralWrist.stationCommand()));

    // L2
    driveStick.a().onTrue(Commands.parallel(elevator.collapseCommand(),
        coralWrist.L2BranchCommand()));
    // driveStick.a().onTrue(elevator.currentPoseCommand());
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

    /* Reset gyro button */
    driveStick.povUp().toggleOnTrue(drivebase.resetGyroCommand());

    driveStick.back().onTrue(
        Commands.sequence(
            elevator.coastModeCommand()));

    /* Align Command Button Logic */
    Trigger semiAutoCancel = new Trigger(this::anyJoystickInput);

    var leftAlignCommand = new LeftReefAlignCommand(drivebase)
        .until(semiAutoCancel)
        // This Smart Dashboard value is used by the CANdleSystem.java subsystem
        .andThen(() -> SmartDashboard.putBoolean("Aligned", false));
    driveStick.leftStick().toggleOnTrue(leftAlignCommand);
    // driveStick.povLeft().toggleOnTrue(leftAlignCommand);

    var rightAlignCommand = new RightReefAlignCommand(drivebase)
        .until(semiAutoCancel)
        // This Smart Dashboard value is used by the CANdleSystem.java subsystem
        .andThen(() -> SmartDashboard.putBoolean("Aligned", false));
    driveStick.rightStick().toggleOnTrue(rightAlignCommand);
    // driveStick.povRight().toggleOnTrue(rightAlignCommand);

    driveStick.povRight().whileTrue(
        Commands.parallel(
            elevator.collapseCommand(),
            coralWrist.highBranchesCommand(),
            coralIntake.intakeCommand()));
  }

  private boolean anyJoystickInput() {
    return deadband(driveStick.getLeftY(), DriveConstants.autoCancelThreshold) != 0
        || deadband(driveStick.getLeftX(), DriveConstants.autoCancelThreshold) != 0
        || deadband(driveStick.getRightX(), DriveConstants.autoCancelThreshold) != 0;
  }

  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }
}

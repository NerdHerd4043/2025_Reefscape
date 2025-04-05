// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.ReefAlignCommand;
import frc.robot.commands.RumbleOnIntake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.coral.CoralWrist;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Climber;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.AutoDestinations.ReefSide;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import cowlib.Util;

@Logged
public class RobotContainer {
  @NotLogged
  private final CommandXboxController driveStick = new CommandXboxController(0);

  private final Drivebase drivebase = new Drivebase();
  private final Elevator elevator = new Elevator();
  private final CoralWrist coralWrist = new CoralWrist();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Climber climber = new Climber();

  @SuppressWarnings("unused")
  private final CANdleSystem CANdle = new CANdleSystem();

  private SendableChooser<Command> autoChooser;

  private Command fixCoral = Commands.sequence(
      coralIntake.outtakeCommand().withTimeout(0.1),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.05),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.05),
      coralIntake.intakeCommand().withTimeout(0.25));

  Command lowAlgaeCommand = Commands.parallel(
      elevator.collapseCommand(),
      coralWrist.highBranchesCommand(),
      coralIntake.intakeCommand().withTimeout(3));

  Command highAlgaeCommand = Commands.parallel(
      elevator.extendCommand(2),
      coralWrist.highBranchesCommand(),
      coralIntake.intakeCommand().withTimeout(1.1));

  Command netScoreCommand = Commands.sequence(
      elevator.extendCommand(4),
      coralWrist.L2BranchCommand(),
      Commands.waitUntil(
          () -> elevator.encoderPosition() > Constants.Elevator.maxElevatorHeight * .8),
      coralIntake.outtakeCommand().withTimeout(2),
      elevator.collapseCommand());

  public boolean climberMode;

  public RobotContainer() {
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            this::getScaledXY,
            () -> scaleRotationAxis(driveStick.getRightX())));

    // climber.setDefaultCommand(climber.run(() -> climber.setSpeed(
    // driveStick.getLeftTriggerAxis() - driveStick.getRightTriggerAxis())));

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

    NamedCommands.registerCommand("Reef Align", new ReefAlignCommand(drivebase, ReefSide.LEFT));

    NamedCommands.registerCommand("Conditional Intake", new ConditionalIntake(coralIntake, elevator));

    NamedCommands.registerCommand("No Drive", new NoDrive(drivebase));

    NamedCommands.registerCommand("Elevator L4", Commands.parallel(
        elevator.extendCommand(4),
        coralWrist.highBranchesCommand()));

    NamedCommands.registerCommand("Score", Commands.sequence(
        Commands.waitSeconds(1),
        coralIntake.outtakeCommand().withTimeout(0.5),
        elevator.collapseCommand(),
        coralWrist.stationCommand()));

    NamedCommands.registerCommand("Straighten Coral", this.fixCoral);

    NamedCommands.registerCommand("Low Algae", lowAlgaeCommand);

    NamedCommands.registerCommand("High Algae", highAlgaeCommand);

    NamedCommands.registerCommand("Net Score", netScoreCommand);

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    CameraServer.startAutomaticCapture(0);

    this.climberMode = false;

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

  private void enterClimberMode() {
    climber.setDefaultCommand(climber.run(() -> climber.setSpeed(
        driveStick.getLeftTriggerAxis() - driveStick.getRightTriggerAxis())));
    coralWrist.highBranches();
    this.climberMode = true;
    SmartDashboard.putBoolean("Climber Mode", this.climberMode);
  }

  private void exitClimberMode() {
    climber.removeDefaultCommand();
    climber.stopCommand().schedule();
    this.climberMode = false;
    SmartDashboard.putBoolean("Climber Mode", this.climberMode);
  }

  public void toggleClimberMode() {
    if (this.climberMode) {
      exitClimberMode();
    } else {
      enterClimberMode();
    }
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

    // FIXME
    driveStick.povDown().onTrue(this.lowAlgaeCommand);
    // driveStick.povDown().onTrue(this.fixCoral);

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
            Commands.either(
                coralWrist.highBranchesCommand(),
                coralWrist.L2BranchCommand(),
                () -> SmartDashboard.getBoolean("Piece Acquired", true))));

    /* Reset gyro button */
    driveStick.povUp().toggleOnTrue(drivebase.resetGyroCommand());

    driveStick.back().whileTrue(elevator.coastModeCommand());

    Trigger leftTriggerLow = driveStick.leftTrigger(0.1);
    Trigger leftTriggerHigh = driveStick.leftTrigger(0.9);

    Trigger climberMode = new Trigger(() -> this.climberMode);

    leftTriggerLow
        .and(leftTriggerHigh.negate())
        .and(climberMode.negate()).whileTrue(Commands.parallel(
            elevator.collapseCommand(),
            coralWrist.highBranchesCommand(),
            coralIntake.intakeCommand()));

    leftTriggerHigh
        .and(climberMode.negate())
        .whileTrue(Commands.parallel(
            elevator.extendCommand(2),
            coralWrist.highBranchesCommand(),
            coralIntake.intakeCommand()));

    driveStick.rightTrigger()
        .and(climberMode.negate())
        .onTrue(Commands.sequence(
            elevator.extendCommand(4),
            coralWrist.L2BranchCommand(),
            Commands.waitUntil(
                () -> elevator.encoderPosition() > Constants.Elevator.maxElevatorHeight * .8),
            coralIntake.outtakeCommand().withTimeout(2)));

    /* Align Command Button Logic */
    Trigger semiAutoCancel = new Trigger(this::anyJoystickInput);

    var fullRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0));

    var leftRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0));
    var leftAlignCommand = Commands.parallel(
        leftRumbleCommand.withTimeout(0.5),
        new ReefAlignCommand(drivebase, ReefSide.LEFT))
        .until(semiAutoCancel)
        // This Smart Dashboard value is used by the CANdleSystem.java subsystem
        .andThen(() -> SmartDashboard.putBoolean("Aligned", false));
    driveStick.leftStick().toggleOnTrue(leftAlignCommand);
    driveStick.povLeft().toggleOnTrue(leftAlignCommand);

    driveStick.start().onTrue(Commands.parallel(
        Commands.runOnce(() -> this.toggleClimberMode()),
        fullRumbleCommand.withTimeout(0.5)));

    var rightRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0));
    var rightAlignCommand = Commands.parallel(
        rightRumbleCommand.withTimeout(0.5),
        new ReefAlignCommand(drivebase, ReefSide.RIGHT))
        .until(semiAutoCancel)
        // This Smart Dashboard value is used by the CANdleSystem.java subsystem
        .andThen(() -> SmartDashboard.putBoolean("Aligned", false));
    driveStick.rightStick().toggleOnTrue(rightAlignCommand);
    driveStick.povRight().toggleOnTrue(rightAlignCommand);
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

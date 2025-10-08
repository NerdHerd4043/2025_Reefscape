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
import frc.robot.commands.AlgaeReefAlignCommand;
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
  // Creates our controller
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
  @NotLogged
  private final CommandXboxController driveStick = new CommandXboxController(0);

  // Creates our subsystems
  private final Drivebase drivebase = new Drivebase();
  private final Elevator elevator = new Elevator();
  private final CoralWrist coralWrist = new CoralWrist();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Climber climber = new Climber();

  // Creates our CANdle System, i.e. our LEDs
  @SuppressWarnings("unused")
  private final CANdleSystem CANdle = new CANdleSystem();

  // This, plus some lines below, creates a drop down menu on the dashboard for
  // selecting our auto.
  private SendableChooser<Command> autoChooser;

  // Straightens the coral (game piece) automatically if the coral was recieved at
  // an angle.
  private Command fixCoral = Commands.sequence(
      coralIntake.outtakeCommand().withTimeout(0.1),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.05),
      coralIntake.intakeCommand().withTimeout(0.15),
      coralIntake.outtakeCommand().withTimeout(0.05),
      coralIntake.intakeCommand().withTimeout(0.25));

  // Algae commands are mostly untested
  Command lowAlgaeCommand = Commands.parallel(
      elevator.collapseCommand(),
      coralWrist.highBranchesCommand(),
      coralIntake.intakeCommand().withTimeout(2));

  Command highAlgaeCommand = Commands.parallel(
      elevator.extendCommand(2),
      coralWrist.highBranchesCommand(),
      coralIntake.intakeCommand().withTimeout(2));

  Command netScoreCommand = Commands.sequence(
      elevator.extendCommand(4),
      coralWrist.L2BranchCommand(),
      Commands.waitUntil(
          () -> elevator.encoderPosition() > Constants.Elevator.maxElevatorHeight * .8),
      coralIntake.outtakeCommand().withTimeout(2),
      elevator.collapseCommand());

  // Used to switch between climber/algae controls connected to the controller
  // triggers. The logic is in `this.configureBindings`.
  public boolean climberMode;

  // Tells the robot to drive by default.
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

    // Makes sure the limelights are on the april tag detection pipeline (refer to
    // limelight documentation for more information).
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary
    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.setPipelineIndex("limelight-right", 0);

    // Creating named commands to be used by Pathplanner.
    // https://pathplanner.dev/home.html
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

    NamedCommands.registerCommand("Algae Reef Align", new AlgaeReefAlignCommand(drivebase));

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

    // This, plus some other lines, creates a drop down menu on the dashboard for
    // selecting our auto.
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    CameraServer.startAutomaticCapture(0);

    // Algae controls are the default, so climber mode is false.
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

  // Changes our input -> output from linear to exponential, allowing finer
  // control close to the center without limiting our max output (since 1 is the
  // highest input and 1^2 (the ouput) = 1, so no change to the edges of the
  // input/output)
  private double[] getScaledXY() {
    // Array for storing the x/y inputs from the controller
    double[] xy = new double[2];

    // Assigning inputs to array locations. X and Y are switched because the
    // controller is funky.
    xy[0] = deadband(-driveStick.getLeftY(), DriveConstants.deadband);
    xy[1] = deadband(-driveStick.getLeftX(), DriveConstants.deadband);

    Util.square2DVector(xy);

    // Scales the max drive speed when the elevator is enabled.
    var scaling = drivebase.getDriverMaxVelocity() * this.getElevatorSpeedRatio();
    xy[0] *= scaling;
    xy[1] *= scaling;

    return xy;
  }

  // The function used to scale the drive speed when the elevator is enabled.
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

  // Some of the climber mode logic
  private void enterClimberMode() {
    climber.setDefaultCommand(climber.run(() -> climber.setSpeed(
        driveStick.getLeftTriggerAxis() - driveStick.getRightTriggerAxis())));
    coralWrist.highBranches();
    this.climberMode = true;
    SmartDashboard.putBoolean("Climber Mode", this.climberMode);
  }

  // Some of the climber mode logic
  private void exitClimberMode() {
    climber.removeDefaultCommand();
    climber.stopCommand().schedule();
    this.climberMode = false;
    SmartDashboard.putBoolean("Climber Mode", this.climberMode);
  }

  // Some of the climber mode logic
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
        .onTrue(fixCoral);

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
    // This, plus some lines above, creates a drop down menu on the dashboard for
    // selecting our auto.
    return this.autoChooser.getSelected();
  }
}

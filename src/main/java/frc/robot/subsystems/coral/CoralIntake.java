// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(Constants.CoralIntake.motorId, MotorType.kBrushless);

  private static final TimeOfFlight distanceSensor = new TimeOfFlight(0);

  private SendableChooser<Double> outputSpeedChooser = new SendableChooser<>();

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.CoralIntake.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    this.intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.outputSpeedChooser = new SendableChooser<>();

    this.outputSpeedChooser.setDefaultOption("0.5", 0.5);
    this.outputSpeedChooser.addOption("Original", 0.6);
    this.outputSpeedChooser.addOption("0.4", 0.4);
    this.outputSpeedChooser.addOption("0.4", 0.45);
    this.outputSpeedChooser.addOption("0.4", 0.35);

    SmartDashboard.putData(this.outputSpeedChooser);
  }

  public void runIntake(double intakeMotorSpeed) {
    this.intakeMotor.set(intakeMotorSpeed);
  }

  public void stop() {
    this.intakeMotor.stopMotor();
  }

  public Command intakeCommand() {
    return this.run(() -> this.runIntake(Constants.CoralIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  public Command outtakeCommand() {
    return this.run(() -> this.runIntake(-this.outputSpeedChooser.getSelected()))
        .finallyDo(this::stop);
  }

  public double getIntakeAmps() {
    return this.intakeMotor.getOutputCurrent();
  }

  public double getDistanceSensorRange() {
    if (this.distanceSensor.isRangeValid()) {
      return this.distanceSensor.getRange();
    } else {
      return -1;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    if (this.distanceSensor.isRangeValid() || true) {
      SmartDashboard.putNumber("Bool Distance Sensor", this.getDistanceSensorRange());
    }

    // SmartDashboard.putNumber("Uwu", SmartDashboard.getNumber("Uwu", 0) + 1);

    // SmartDashboard.putNumber("Intake Amps", this.getIntakeAmps());

    // SmartDashboard.putNumber("Distance Sensor", this.getDistanceSensorRange());
  }
}

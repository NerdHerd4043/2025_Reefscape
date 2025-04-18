// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class CoralIntake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(Constants.CoralIntake.motorId, MotorType.kBrushless);

  private final TimeOfFlight distanceSensor = new TimeOfFlight(33);

  // Added this to change output speed mid-match, since it was having some issues
  // with scoring.
  private SendableChooser<Double> outputSpeedChooser = new SendableChooser<>();

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.CoralIntake.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    // Added this to change output speed mid-match, since it was having some issues
    // with scoring.
    this.intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Added this to change output speed mid-match, since it was having some issues
    // with scoring.
    this.outputSpeedChooser = new SendableChooser<>();

    // Added this to change output speed mid-match, since it was having some issues
    // with scoring.
    this.outputSpeedChooser.setDefaultOption("0.5", 0.5);
    this.outputSpeedChooser.addOption("Original", 0.6);
    this.outputSpeedChooser.addOption("0.4", 0.4);
    this.outputSpeedChooser.addOption("0.4", 0.45);
    this.outputSpeedChooser.addOption("0.4", 0.35);

    // Added this to change output speed mid-match, since it was having some issues
    // with scoring.
    SmartDashboard.putData("Outtake Speed", this.outputSpeedChooser);
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

  // Returns the power being supplied to the intake motor.
  public double getIntakeAmps() {
    return this.intakeMotor.getOutputCurrent();
  }

  // Returns the distance sensor measurement.
  public double getDistanceSensorRange() {
    return this.distanceSensor.getRange();
  }

  public boolean pieceAquired() {
    return this.distanceSensor.getRange() < Constants.CoralIntake.distSensorHighNoCoral - 20;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.

    // Puts the distance sensor measurement onto the dashboard if there is a
    // measurement.
    if (this.distanceSensor.isRangeValid()) {
      SmartDashboard.putNumber("Distance Sensor", this.getDistanceSensorRange());
    }

    // This Smart Dashboard value is used by the CANdleSystem.java subsystems
    if (!pieceAquired()) {
      SmartDashboard.putBoolean("Aligned", false);
    }

    SmartDashboard.putBoolean("Piece Acquired", this.pieceAquired());

    // SmartDashboard.putNumber("Intake Amps", this.getIntakeAmps());
  }
}

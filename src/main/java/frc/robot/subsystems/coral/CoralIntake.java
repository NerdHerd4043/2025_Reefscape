// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private SparkMax intakeMotor = new SparkMax(Constants.CoralIntake.motorId, MotorType.kBrushless);

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(40);

    intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake(double intakeMotorSpeed) {
    intakeMotor.set(intakeMotorSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public Command intakeCommand() {
    return this.run(() -> this.runIntake(Constants.CoralIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  public Command outtakeCommand() {
    return this.run(() -> this.runIntake(-Constants.CoralIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

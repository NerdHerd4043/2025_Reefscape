// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(Constants.CoralIntake.motorId, MotorType.kBrushless);

  private Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.CoralIntake.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    this.intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.distanceSensor.setAutomaticMode(true);
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
    return this.run(() -> this.runIntake(-Constants.CoralIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  public double getIntakeAmps() {
    return this.intakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    if (this.distanceSensor.isRangeValid() || true) {
      SmartDashboard.putNumber("Distance Sensor", this.distanceSensor.getRange());
    }

    SmartDashboard.putNumber("Uwu", SmartDashboard.getNumber("Uwu", 0) + 1);

    SmartDashboard.putNumber("Intake Amps", this.getIntakeAmps());
  }
}

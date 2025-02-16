// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
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
  private SparkMax intakeMotor = new SparkMax(Constants.CoralIntake.motorId, MotorType.kBrushless);

  private final Rev2mDistanceSensor distanceSensor;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(40);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    this.distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    this.distanceSensor.setAutomaticMode(true);

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

  public double getIntakeAmps() {
    return intakeMotor.getOutputCurrent();
  }

  public double getDistSensor() {
    if (this.distanceSensor.isRangeValid()) {
      return this.distanceSensor.GetRange();
    } else {
      return 999999.99;
    }
  }

  public boolean coralAquired() {
    if (this.getDistSensor() < 0.675) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Amps", getIntakeAmps());

    SmartDashboard.putNumber("Distance Sensor", getDistSensor());

  }
}

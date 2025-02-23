// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

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

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(Constants.AlgaeIntake.motorID, MotorType.kBrushless);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.AlgaeIntake.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    this.intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake(double intakeMotorSpeed) {
    this.intakeMotor.set(intakeMotorSpeed);
  }

  public void stop() {
    this.intakeMotor.stopMotor();
  }

  public Command intakeCommand() {
    return this.run(() -> runIntake(Constants.AlgaeIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  public Command outtakeCommand() {
    return this.run(() -> runIntake(-Constants.AlgaeIntake.intakeSpeed))
        .finallyDo(this::stop);
  }

  private double getAlgaeIntakeAmps() {
    return intakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Algae Intake Amps", getAlgaeIntakeAmps());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeWrist.PIDValuesA;

public class AlgaeWrist extends SubsystemBase {
  final SparkMax wristMotor = new SparkMax(Constants.AlgaeWrist.motorID, MotorType.kBrushless);

  private ArmFeedforward feedforward = new ArmFeedforward(
      Constants.AlgaeWrist.FeedForwardValuesA.kS,
      Constants.AlgaeWrist.FeedForwardValuesA.kG,
      Constants.AlgaeWrist.FeedForwardValuesA.kV);

  private double ffOutput;

  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesA.p,
      PIDValuesA.i,
      PIDValuesA.d,
      Constants.AlgaeWrist.constraintsA);

  /** Creates a new AlgaeWrist. */
  public AlgaeWrist() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.AlgaeWrist.currentLimit);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

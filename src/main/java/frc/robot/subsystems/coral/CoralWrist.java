// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralWrist.PIDValues;

public class CoralWrist extends SubsystemBase {
  private SparkMax wristMotor = new SparkMax(Constants.CoralWrist.motorId, MotorType.kBrushless);

  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0); // FIXME: Tune

  private double ffOutput;

  // FIXME: Tune
  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValues.p,
      PIDValues.i,
      PIDValues.d,
      new TrapezoidProfile.Constraints(
          0, 0)); // FIXME: Tune

  /** Creates a new CoralWrist. */
  public CoralWrist() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(40);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

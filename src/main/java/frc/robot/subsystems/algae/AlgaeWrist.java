// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeWrist.PIDValuesA;
import frc.robot.Constants.AlgaeWrist.WristPositionsA;

public class AlgaeWrist extends SubsystemBase {
  final SparkMax wristMotor = new SparkMax(Constants.AlgaeWrist.motorID, MotorType.kBrushless);

  private ArmFeedforward feedforward = new ArmFeedforward(
      Constants.AlgaeWrist.FeedForwardValuesA.kS,
      Constants.AlgaeWrist.FeedForwardValuesA.kG,
      Constants.AlgaeWrist.FeedForwardValuesA.kV);

  private double ffOutput;

  private RelativeEncoder encoder = wristMotor.getEncoder();

  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesA.p,
      PIDValuesA.i,
      PIDValuesA.d,
      Constants.AlgaeWrist.constraintsA);

  /** Creates a new AlgaeWrist. */
  public AlgaeWrist() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.AlgaeWrist.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.pidController.setGoal(getEncoder());
  }

  private void updatePID() {
    var setpoint = getSetpoint();
    var ff = -feedforward.calculate(setpoint.position, setpoint.velocity);
    wristMotor.setVoltage(ff + pidController.calculate(getEncoder()));
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public void setTarget(double target) {
    this.pidController.setGoal(MathUtil.clamp(target, WristPositionsA.lower, WristPositionsA.upper));
  }

  public double getEncoder() {
    return this.encoder.getPosition();
  }

  public void resetPosition() {
    this.encoder.setPosition(0);
  }

  public void up() {
    this.pidController.setGoal(WristPositionsA.upper);
  }

  public void down() {
    this.pidController.setGoal(WristPositionsA.lower);
  }

  public Command getUpCommand() {
    return this.run(() -> this.up());
  }

  public Command getDownCommand() {
    return this.run(() -> this.down());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePID();
    // if (wristMotor.getForwardLimitSwitch().isPressed()) {
    // resetPosition();
    // }
    SmartDashboard.putNumber("Algae Wrist Encoder", getEncoder());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralWrist.PIDValuesC;
import frc.robot.Constants.CoralWrist.WristPositions;

@Logged
public class CoralWrist extends SubsystemBase {
  @NotLogged
  private SparkMax wristMotor = new SparkMax(Constants.CoralWrist.motorId, MotorType.kBrushless);

  @Logged
  private CANcoder encoder = new CANcoder(Constants.CoralWrist.encoderID); // FIXME: Set ID

  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0); // FIXME: Tune

  private double ffOutput;

  private boolean enabled = false;

  // FIXME: Tune
  @Logged
  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesC.p,
      PIDValuesC.i,
      PIDValuesC.d,
      Constants.CoralWrist.constraints); // FIXME: Tune

  /** Creates a new CoralWrist. */
  public CoralWrist() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.CoralWrist.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);

    wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.pidController.setGoal(getEncoderRadians());
  }

  private void updatePID() {
    var setpoint = getSetpoint();
    ffOutput = -feedforward.calculate(setpoint.position, setpoint.velocity);
    this.wristMotor.setVoltage(ffOutput + pidController.calculate(getEncoderRadians()));
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public void setTarget(double target) {
    personalSetGoal(target);
  }

  public void station() {
    personalSetGoal(WristPositions.stationPos);
  }

  public void L2Branch() {
    personalSetGoal(WristPositions.L2BranchPos);
  }

  public void highBranches() {
    personalSetGoal(WristPositions.highBranchesPos);
  }

  public double getEncoder() {
    return -this.encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return getEncoder() * 2 * Math.PI;
  }

  public Command getL2BranchCommand() {
    this.enabled = true;
    return this.runOnce(() -> this.L2Branch());
  }

  public Command getHighBranchesCommand() {
    this.enabled = true;
    return this.runOnce(() -> this.highBranches());
  }

  public Command getStationCommand() {
    this.enabled = true;
    return this.runOnce(() -> this.station());
  }

  private void personalSetGoal(double input) {
    this.pidController.setGoal(
        MathUtil.clamp(input, 0.0, WristPositions.upper));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updatePID();

    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint().position);
    SmartDashboard.putNumber("Wrist Goal", this.pidController.getGoal().position);
    SmartDashboard.putNumber("Coral Wrist Encoder", getEncoderRadians());
  }
}

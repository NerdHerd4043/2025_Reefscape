// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.Climber.PIDValuesC;
import frc.robot.Constants.CoralWrist.WristPositionsC;
import frc.robot.Constants.Climber.ClimberPositionsC;

@Logged
public class Climber extends SubsystemBase {
  @NotLogged
  private final SparkMax wristMotor = new SparkMax(Constants.Climber.motorId, MotorType.kBrushless);

  @Logged
  private CANcoder encoder = new CANcoder(Constants.Climber.encoderID); // FIXME: Set ID

  // FIXME: Tune
  @Logged
  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesC.p,
      PIDValuesC.i,
      PIDValuesC.d,
      Constants.Climber.constraints); // FIXME: Tune

  /** Creates a new Climber. */
  public Climber() {
    final SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(Constants.Climber.currentLimit);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(false);

    this.wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.pidController.setGoal(this.getEncoderRadians());
  }

  private void updatePID() {
    this.wristMotor.setVoltage(pidController.calculate(getEncoderRadians()));
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public Command stationCommand() {
    return this.runOnce(this::station);
  }

  public Command highPosCommand() {
    return this.runOnce(this::setHighPos);
  }

  public Command highLowCommand() {
    return this.runOnce(this::setLowPos);
  }

  public void station() {
    this.setGoal(ClimberPositionsC.downPos);
  }

  public void setHighPos() {
    this.setGoal(ClimberPositionsC.upPos);
  }

  public void setLowPos() {
    this.setGoal(ClimberPositionsC.downPos);
  }

  public void setTarget(double target) {
    this.setGoal(target);
  }

  public void setSpeed(double speed) {
    this.wristMotor.set(speed);
  }

  public double getEncoder() {
    return -this.encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderRadians() {
    return this.getEncoder() * 2 * Math.PI;
  }

 
  private void setGoal(double input) {
    this.pidController.setGoal(
        MathUtil.clamp(input, 0.0, ClimberPositionsC.upper));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // this.updatePID();

    // SmartDashboard.putNumber("Wrist Setpoint", this.getSetpoint().position);
    // SmartDashboard.putNumber("Wrist Goal",
    // this.pidController.getGoal().position);
    // SmartDashboard.putNumber("Coral Wrist Encoder", this.getEncoderRadians());

    // SmartDashboard.putNumber("EYE", this.pidController.getAccumulatedError());
  }
}

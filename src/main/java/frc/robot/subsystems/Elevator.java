// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.Elevator.PIDValues;

@Logged
public class Elevator extends SubsystemBase {
  @NotLogged
  private SparkMax leftMotor = new SparkMax(Constants.Elevator.leftMotorId, MotorType.kBrushless);
  @NotLogged
  private SparkMax rightMotor = new SparkMax(Constants.Elevator.rightMotorId, MotorType.kBrushless);

  private boolean positionKnown = false;

  private boolean enabled = false;
  private boolean extended = false;

  private RelativeEncoder encoder = rightMotor.getEncoder();
  private SparkLimitSwitch limitSwitch;

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(
      Constants.Elevator.FeedForwardValues.kS,
      Constants.Elevator.FeedForwardValues.kG,
      Constants.Elevator.FeedForwardValues.kV);

  @Logged
  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValues.p,
      PIDValues.i,
      PIDValues.d,
      // The motion profile constraints
      Constants.Elevator.constraints);

  /** Creates a new Elevator. */
  public Elevator() {
    final SparkMaxConfig baseMotorConfig = new SparkMaxConfig();

    baseMotorConfig.smartCurrentLimit(Constants.Elevator.currentLimit);
    baseMotorConfig.idleMode(IdleMode.kBrake);

    final SparkMaxConfig leftMotorConfig = new SparkMaxConfig().apply(baseMotorConfig);
    final SparkMaxConfig rightMotorConfig = new SparkMaxConfig().apply(baseMotorConfig);

    leftMotorConfig.follow(Constants.Elevator.rightMotorId, true);
    rightMotorConfig.inverted(true);

    rightMotorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    this.leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.limitSwitch = this.rightMotor.getReverseLimitSwitch();

    if (this.limitSwitchPressed()) {
      this.resetPosition();
    }
  }

  private void updatePID() {
    var setpoint = this.getSetpoint();
    var ff = this.feedforward.calculate(setpoint.position, setpoint.velocity);
    this.rightMotor.setVoltage(ff + this.pidController.calculate(this.encoderPosition()));
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public double getEncoder() {
    return this.encoder.getPosition();
  }

  public void resetPosition() {
    this.positionKnown = true;
    this.encoder.setPosition(0);
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public void extend(int level) {
    this.enable();
    this.extended = true;
    this.setGoal(Constants.Elevator.elevatorHeights[level]);
  }

  public void collapse() {
    this.extended = false;
    this.setGoal(Constants.Elevator.elevatorHeights[0]);
  }

  // For testing purposes, not used in final robot
  public void currentPose() {
    this.setGoal(this.encoderPosition());
  }

  public Command getExtendCommand(int level) {
    return this.runOnce(() -> this.extend(level));
  }

  public Command getCollapseCommand() {
    return this.runOnce(this::collapse);
  }

  public Command getCurrentPoseCommand() {
  // For testing purposes, not used in final robot
    return this.runOnce(this::currentPose);
  }

  public boolean isElevatorExtended() {
    return this.enabled;
  }

  private void personalSetGoal(double input) {
    this.pidController.setGoal(
        MathUtil.clamp(input, 0, Constants.Elevator.maxElevatorHeight));
  }

  public boolean limitSwitchPressed() {
    return this.limitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    if (!this.extended && this.limitSwitchPressed()) {
      this.disable();
      this.resetPosition();
    }
    if (this.enabled /* && this.positionKnown */) {
      this.updatePID();
    }

    SmartDashboard.putNumber("Elevator Encoder", this.encoderPosition());

    SmartDashboard.putNumber("Setpoint", this.pidController.getSetpoint().position);

    SmartDashboard.putBoolean("Elevator Limit Switch", this.limitSwitchPressed());

    SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
    SmartDashboard.putBoolean("Elevator Extended", this.extended);
  }
}

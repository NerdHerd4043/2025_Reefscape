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

import frc.robot.Constants.Elevator.PIDValuesE;

@Logged
public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor = new SparkMax(Constants.Elevator.leftMotorId, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(Constants.Elevator.rightMotorId, MotorType.kBrushless);

  private boolean positionKnown = false;

  private boolean enabled = false;
  private boolean extended = false;

  private RelativeEncoder encoder = rightMotor.getEncoder();
  private SparkLimitSwitch limitSwitch;

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(
      Constants.Elevator.FeedForwardValues.kS,
      Constants.Elevator.FeedForwardValues.kG,
      Constants.Elevator.FeedForwardValues.kV);

  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesE.p,
      PIDValuesE.i,
      PIDValuesE.d,
      // The motion profile constraints
      Constants.Elevator.constraints);

  @NotLogged
  final SparkMaxConfig leftMotorConfigBrake;
  @NotLogged
  final SparkMaxConfig rightMotorConfigBrake;

  @NotLogged
  final SparkMaxConfig leftMotorConfigCoast;
  @NotLogged
  final SparkMaxConfig rightMotorConfigCoast;

  /** Creates a new Elevator. */
  public Elevator() {
    final SparkMaxConfig baseMotorConfig = new SparkMaxConfig();

    baseMotorConfig.smartCurrentLimit(Constants.Elevator.currentLimit);
    baseMotorConfig.idleMode(IdleMode.kBrake);

    leftMotorConfigBrake = new SparkMaxConfig().apply(baseMotorConfig);
    rightMotorConfigBrake = new SparkMaxConfig().apply(baseMotorConfig);

    leftMotorConfigBrake.follow(Constants.Elevator.rightMotorId, true);
    rightMotorConfigBrake.inverted(true);

    rightMotorConfigBrake.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    rightMotorConfigCoast = new SparkMaxConfig().apply(rightMotorConfigBrake);
    leftMotorConfigCoast = new SparkMaxConfig().apply(leftMotorConfigBrake);

    rightMotorConfigCoast.idleMode(IdleMode.kCoast);
    leftMotorConfigCoast.idleMode(IdleMode.kCoast);

    this.leftMotor.configure(leftMotorConfigBrake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.rightMotor.configure(rightMotorConfigBrake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.limitSwitch = this.rightMotor.getReverseLimitSwitch();

    if (this.limitSwitch.isPressed()) {
      this.resetPosition();
    }
  }

  public Command holdCommand() {
    return this.runOnce(() -> this.pidController.setGoal(this.encoderPosition()));
  }

  public Command coastModeCommand() {
    return this.startEnd(() -> {
      this.leftMotor.configure(leftMotorConfigCoast, ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      this.rightMotor.configure(rightMotorConfigCoast, ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }, () -> {
      this.leftMotor.configure(leftMotorConfigBrake, ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
      this.rightMotor.configure(rightMotorConfigBrake, ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
    }).ignoringDisable(true);
  }

  private void updatePID() {
    var setpoint = this.getSetpoint();
    @SuppressWarnings("removal")
    var ff = this.feedforward.calculate(setpoint.position, setpoint.velocity);
    var output = ff + this.pidController.calculate(this.encoderPosition());
    SmartDashboard.putNumber("Elevator Output", output);
    this.rightMotor.setVoltage(output);
  }

  private void stopMotor() {
    this.rightMotor.stopMotor();
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public double encoderPosition() {
    return this.encoder.getPosition();
  }

  public void resetPosition() {
    if (!this.positionKnown || this.encoderPosition() < 4) {
      this.positionKnown = true;
      this.encoder.setPosition(0);
    }
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

  public Command extendCommand(int level) {
    return this.runOnce(() -> this.extend(level));
  }

  public Command collapseCommand() {
    return this.runOnce(this::collapse);
  }

  // For testing purposes, not used in final robot
  public Command currentPoseCommand() {
    return this.runOnce(this::currentPose);
  }

  public boolean elevatorExtended() {
    return this.enabled;
  }

  private void setGoal(double input) {
    this.pidController.setGoal(
        MathUtil.clamp(input, 0, Constants.Elevator.maxElevatorHeight));
  }

  public boolean limitSwitchPressed() {
    return this.limitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    if (!this.extended && this.limitSwitchPressed()) {
      this.stopMotor();
      this.disable();
      this.resetPosition();
    }
    if (this.enabled /* && this.positionKnown */) {
      this.updatePID();
    }

    SmartDashboard.putNumber("Elevator Setpoint", this.pidController.getSetpoint().position);
    SmartDashboard.putNumber("Elevator Applied Output", this.rightMotor.getAppliedOutput());

    // SmartDashboard.putBoolean("Elevator Limit Switch",
    // this.limitSwitchPressed());

    // SmartDashboard.putBoolean("Elevator Enabled", this.enabled);
    // SmartDashboard.putBoolean("Elevator Extended", this.extended);
  }
}

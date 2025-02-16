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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import frc.robot.Constants.Elevator.PIDValuesE;

@Logged
public class Elevator extends SubsystemBase {
  @NotLogged
  private final SparkMax leftMotor = new SparkMax(Constants.Elevator.leftMotorId, MotorType.kBrushless);
  @NotLogged
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

  @Logged
  private ProfiledPIDController pidController = new ProfiledPIDController(
      PIDValuesE.p,
      PIDValuesE.i,
      PIDValuesE.d,
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

    // I only keep this because I'm not certain if when starting
    // the robot with the limit switch already pressed will still
    // cause the trigger to schedule the reset command
    if (limitSwitch.isPressed()) {
      resetPosition();
    }

    // You should already be familiar with Triggers.
    // In `RobotContainer` we use methods on `CommandXboxController` to get Triggers
    // and bind commands to button press
    Trigger limitSwitchTrigger = new Trigger(this::limitSwitchPressed);
    // Could implement `this::isCollapsed` if we need to get the collapsed state
    // again later
    Trigger collapsedTrigger = new Trigger(() -> !this.extended);

    // Just like in robot container, we can compose some triggers to schedule
    // a command, we just do it here because I arbitrarily decided that is the
    // responsibility of the elevator itself
    limitSwitchTrigger
        .and(collapsedTrigger)
        .onTrue(this.resetPositionCommand());
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

  public boolean limitSwitchPressed() {
    return this.limitSwitch.isPressed();
  }

  public double encoderPosition() {
    return this.encoder.getPosition();
  }

  // We use the term `zero` a lot when talking about this
  // not really sure if that name works better or not
  public void resetPosition() {
    // I moved disable here because it made this api a bit more concise elsewhere,
    // the only change in functionality is that the pidloop, which is already
    // disabled, will get re-disabled during that initial limit switch check
    this.disable();
    this.positionKnown = true;
    this.encoder.setPosition(0);
  }

  // Would be `zeroCommand`
  public Command resetPositionCommand() {
    return this.runOnce(this::resetPosition);
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

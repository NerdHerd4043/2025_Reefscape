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

    // FIXME: need a limit switch wired
    // rightMotorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetPosition();
    this.collapse();
  }

  private void updatePID() {
    var setpoint = getSetpoint();
    var ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    rightMotor.setVoltage(ff + pidController.calculate(getEncoder()));
  }

  @NotLogged
  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public double getEncoder() {
    return this.encoder.getPosition();
  }

  public void resetPosition() {
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
    this.pidController.setGoal(Constants.Elevator.elevatorHeights[level]);
  }

  public void collapse() {
    this.extended = false;
    this.pidController.setGoal(Constants.Elevator.elevatorHeights[0]);
  }

  // For testing purposes
  public void currentPose() {
    this.pidController.setGoal(getEncoder());
  }

  public Command getExtendCommand(int level) {
    return this.runOnce(() -> extend(level));
  }

  public Command getCollapseCommand() {
    return this.runOnce(this::collapse);
  }

  public Command getCurrentPoseCommand() {
    return this.runOnce(this::currentPose);
  }

  // public boolean isElevatorExtended() {
  // return this.extended;
  // }

  @Override
  public void periodic() {
    // FIXME: uncomment when LimitSwitch is wired
    // if (!this.extended && rightMotor.getReverseLimitSwitch().isPressed()) {
    // this.disable();
    // resetPosition();
    // }

    updatePID();

    SmartDashboard.putNumber("Elevator Encoder", getEncoder());

    SmartDashboard.putNumber("Setpoint", this.pidController.getSetpoint().position);

    double[] elevatorUwu = { this.pidController.getSetpoint().position, getEncoder() };
    SmartDashboard.putNumberArray("Elevator Uwu", elevatorUwu);

    SmartDashboard.putBoolean("Limit", rightMotor.getReverseLimitSwitch().isPressed());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.Elevator.PIDValues;

@Logged
public class Elevator extends SubsystemBase {
  private SparkMax leftMotor = new SparkMax(Constants.Elevator.leftMotorId, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Constants.Elevator.rightMotorId, MotorType.kBrushless);

  private boolean enabled = false;

  @Logged
  private RelativeEncoder encoder = rightMotor.getEncoder();

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

    // FIXME: need a limit switch wired
    // rightMotorConfig.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void updatePID() {
    var setpoint = getSetpoint();
    var ff = -feedforward.calculate(setpoint.position, setpoint.velocity);
    rightMotor.setVoltage(ff + pidController.calculate(getMeasurement()));
  }

  public TrapezoidProfile.State getSetpoint() {
    return this.pidController.getSetpoint();
  }

  public double getMeasurement() {
    return this.encoder.getPosition();
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  @Override
  public void periodic() {
    if (this.enabled) {
      updatePID();
    }
  }
}

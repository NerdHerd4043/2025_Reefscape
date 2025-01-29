// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private RelativeEncoder speedEncoder;
  private PIDController pidController;
  private CANcoder encoder;
  private double maxVelocity;
  private double maxVoltage;

  public SwerveModule(int angleMotorId, int speedMotorId, int encoderId, boolean driveInverted,
      double maxVelocity, double maxVoltage) {
    this.angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
    this.speedMotor = new CANSparkMax(speedMotorId, MotorType.kBrushless);

    this.speedMotor.setIdleMode(IdleMode.kBrake);
    this.angleMotor.setIdleMode(IdleMode.kBrake);

    this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
    this.encoder = new CANcoder(encoderId);
    this.maxVelocity = maxVelocity;
    this.maxVoltage = maxVoltage;

    this.pidController.enableContinuousInput(-180, 180);

    this.speedMotor.setInverted(driveInverted);

    // Set scaling factors
    this.speedEncoder = this.speedMotor.getEncoder();
    double driveReduction = 1.0 / 6.75;
    double WHEEL_DIAMETER = 0.1016;
    double rotationsToDistance = driveReduction * WHEEL_DIAMETER * Math.PI;

    this.speedEncoder.setPositionConversionFactor(rotationsToDistance);
    this.speedEncoder.setVelocityConversionFactor(rotationsToDistance / 60);
  }

  public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
    this(config.angleMotorId,
        config.driveMotorId,
        config.encoderId,
        config.drive_inverted,
        maxVelocity,
        maxVoltage);

    angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
  }

  private void drive(double speedMetersPerSecond, double angle) {
    double voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
    speedMotor.setVoltage(voltage);
    angleMotor.setVoltage(-pidController.calculate(this.getEncoder(), angle));
  }

  public void drive(SwerveModuleState state) {
    SwerveModuleState optimized = SwerveModuleState.optimize(state, new Rotation2d(getEncoderRadians()));
    this.drive(optimized.speedMetersPerSecond, optimized.angle.getDegrees());
  }

  public double getEncoder() {
    return encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public double getDriveOutput() {
    return speedMotor.getAppliedOutput();
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getEncoder());
  }

  public double getEncoderRadians() {
    return Units.degreesToRadians(getEncoder());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(speedEncoder.getPosition(), getRotation());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(speedEncoder.getVelocity(), getRotation());
  }
}

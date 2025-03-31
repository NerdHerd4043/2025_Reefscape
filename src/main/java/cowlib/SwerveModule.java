// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
@Logged
public class SwerveModule {
  private SparkMax angleMotor;
  private SparkMax speedMotor;
  private RelativeEncoder speedEncoder;
  private PIDController pidController;
  private CANcoder encoder;
  private double maxVelocity;
  private double maxVoltage;

  public SwerveModule(int angleMotorId, int speedMotorId, int encoderId, boolean driveInverted,
      double maxVelocity, double maxVoltage) {
    this.angleMotor = new SparkMax(angleMotorId, MotorType.kBrushless);
    this.speedMotor = new SparkMax(speedMotorId, MotorType.kBrushless);

    final SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
    final SparkMaxConfig speedMotorConfig = new SparkMaxConfig();

    speedMotorConfig.inverted(driveInverted);
    speedMotorConfig.idleMode(IdleMode.kBrake);

    double driveReduction = 1.0 / 6.12;
    double WHEEL_DIAMETER = 0.1016;
    double rotationsToDistance = driveReduction * WHEEL_DIAMETER * Math.PI;

    speedMotorConfig.encoder
        .positionConversionFactor(rotationsToDistance)
        .velocityConversionFactor(rotationsToDistance);

    this.angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.speedMotor.configure(speedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
    this.encoder = new CANcoder(encoderId);
    this.maxVelocity = maxVelocity;
    this.maxVoltage = maxVoltage;

    this.pidController.enableContinuousInput(-180, 180);

    // Set scaling factors
    this.speedEncoder = this.speedMotor.getEncoder();
  }

  public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
    this(config.angleMotorId,
        config.driveMotorId,
        config.encoderId,
        config.drive_inverted,
        maxVelocity,
        maxVoltage);
  }

  private void drive(double speedMetersPerSecond, double angle) {
    double voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
    this.speedMotor.setVoltage(voltage);
    this.angleMotor.setVoltage(-this.pidController.calculate(this.getEncoder(), angle));
  }

  public void drive(SwerveModuleState state) {
    state.optimize(new Rotation2d(getEncoderRadians()));
    state.cosineScale(this.getRotation());
    this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
  }

  public double getEncoder() {
    return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public double getDriveOutput() {
    return this.speedMotor.getAppliedOutput();
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(this.getEncoder());
  }

  public double getEncoderRadians() {
    return Units.degreesToRadians(this.getEncoder());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.speedEncoder.getPosition(), this.getRotation());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.speedEncoder.getVelocity(), this.getRotation());
  }
}

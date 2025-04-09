// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib.logging;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/** Add your docs here. */
@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
  public SparkMaxLogger() {
    super(SparkMax.class);
  }

  @Override
  protected void update(EpilogueBackend backend, SparkMax motor) {
    backend.log("Applied Output (Duty Cycle)", motor.getAppliedOutput());
    backend.log("Output Current (A)", motor.getOutputCurrent());
    backend.log("Temperature (C)", motor.getMotorTemperature());
  }
}

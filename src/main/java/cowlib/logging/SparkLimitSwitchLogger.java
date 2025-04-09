// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib.logging;

import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/** Add your docs here. */
@CustomLoggerFor(SparkLimitSwitch.class)
public class SparkLimitSwitchLogger extends ClassSpecificLogger<SparkLimitSwitch> {
    public SparkLimitSwitchLogger() {
        super(SparkLimitSwitch.class);
    }

    @Override
    protected void update(EpilogueBackend backend, SparkLimitSwitch limitSwitch) {
        backend.log("Limit Switch", limitSwitch.isPressed());
    }
}

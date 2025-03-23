// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib.logging;

import cowlib.SwerveModule;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/** Add your docs here. */
@CustomLoggerFor(SwerveModule.class)
public class SwerveModuleLogger extends ClassSpecificLogger<SwerveModule> {
    public SwerveModuleLogger() {
        super(SwerveModule.class);
    }

    @Override
    protected void update(EpilogueBackend backend, SwerveModule object) {
        backend.log("Current Angle (Degrees)", object.getEncoder());
        backend.log("Drive Voltage (V)", object.getDriveOutput());
    }
}

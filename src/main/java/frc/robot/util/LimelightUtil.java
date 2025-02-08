// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;

import frc.robot.util.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class LimelightUtil {
    public double offset;
    public int ID;
    public int myID;

    private Map<Integer, OffsetRatios> IDRatios = Map.of(
            3, new OffsetRatios(1.0, 3.0),
            4, new OffsetRatios(3, 2)); // Imagine there are more items in the map

    public double getTargetX(double offset) {
        this.offset = offset;
        this.ID = getID();

        final double xRatio = this.IDRatios.get(this.ID).xRatio;

        return this.offset * xRatio;
    }

    public double getTargetY(double offset) {
        this.offset = offset;
        this.ID = getID();

        final double yRatio = this.IDRatios.get(this.ID).yRatio;

        return this.offset * yRatio;
    }

    public int getID() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-one");
        for (RawFiducial fiducial : fiducials) {
            int myID = fiducial.id;
        }

        return myID;
    }
}

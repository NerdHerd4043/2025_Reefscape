// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class LimelightUtil {
    public double offset;
    public int ID;
    public int myID;

    private Map<Integer, OffsetRatios> IDRatiosMap = Map.of(
            3, new OffsetRatios(1.0, 3.0),
            4, new OffsetRatios(3, 2)); // Imagine there are more items in the map

    public double getTargetX(double offset) {
        this.offset = offset;
        // this.ID = getID();

        final double xRatio = this.IDRatiosMap.get(this.ID).xRatio;

        return this.offset * xRatio;
    }

    public double getTargetY(double offset) {
        this.offset = offset;
        // this.ID = getID();

        final double yRatio = this.IDRatiosMap.get(this.ID).yRatio;

        return this.offset * yRatio;
    }

    public int getID() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-one");
        for (RawFiducial fiducial : fiducials) {
            int myID = fiducial.id;
        }

        return myID;
    }

    public static Pose2d getRobotPose_FieldSpace2D(double yaw) {
        // FIXME: set IMU mode elsewhere so it isn't set each time we get our position
        LimelightHelpers.SetIMUMode("limelight-one", 2);

        int[] validIDs = { 18 }; // FIXME: add all valid ids
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-one", validIDs);

        LimelightHelpers.SetRobotOrientation("limelight-one", yaw, 0, 0, 0, 0, 0);

        Pose2d pos = LimelightHelpers.getBotPose2d_wpiBlue("limelight-one");
        return pos;
    }

    public static double getX() {
        int[] validIDs = { 18 };
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-one", validIDs);
        double posEstimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one").pose.getX();
        return posEstimate;
    }
}

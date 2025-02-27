// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class LimelightUtil {

    public static int getID(String LL_Name) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LL_Name);
        int[] ID = new int[1];
        for (RawFiducial fiducial : fiducials) {
            ID[0] = fiducial.id;
        }
        return ID[0];
    }

    // FIXME: Test
    public static Pose2d getRobotFieldPose2D(double[] limelightArray, AHRS gyro) {
        Pose2d pose = new Pose2d(
                limelightArray[0], // x
                limelightArray[1], // y
                gyro.getRotation2d()); // Yaw
        return pose;
    }
}

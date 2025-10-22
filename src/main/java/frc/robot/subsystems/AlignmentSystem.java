// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AutoDestinations.ReefSide;
import frc.robot.util.LimelightUtil;

public class AlignmentSystem extends SubsystemBase {
  private static NetworkTable smart_dashboard = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard");
  private static NetworkTable alignment_table = NetworkTableInstance.getDefault()
      .getTable("alignment");

  public static BooleanEntry aligned_entry = alignment_table
      .getBooleanTopic("Aligned").getEntry(false);

  private static BooleanEntry right_aligned_entry = alignment_table
      .getSubTable("right").getBooleanTopic("Aligned").getEntry(false);
  public static BooleanEntry left_aligned_entry = alignment_table
      .getSubTable("left").getBooleanTopic("Aligned").getEntry(false);
  private static BooleanSubscriber piece_sub = smart_dashboard
      .getBooleanTopic("Piece Acquired").subscribe(false);
  private static DoubleSubscriber distance_sensor_sub = smart_dashboard
      .getDoubleTopic("Distance Sensor").subscribe(Constants.CoralIntake.distSensorLow);

  private static DoubleEntry left_pose_sub = alignment_table
      .getSubTable("left").getDoubleTopic("Reef Target").getEntry(0);
  private static DoubleEntry right_pose_sub = alignment_table
      .getSubTable("right").getDoubleTopic("Reef Target").getEntry(0.32);

  private static DoubleEntry max_offset_entry = alignment_table
      .getDoubleTopic("Max Offset").getEntry(1);
  private static DoubleEntry deadband_entry = alignment_table
      .getDoubleTopic("Deadband").getEntry(0.02);

  private static DoubleEntry left_delta_x_entry = alignment_table
      .getSubTable("left").getDoubleTopic("Delta X").getEntry(0);
  private static DoubleEntry right_delta_x_entry = alignment_table
      .getSubTable("right").getDoubleTopic("Delta X").getEntry(0);

  /** Creates a new CANdleSystem. */
  public AlignmentSystem() {
    max_offset_entry.setDefault(1);
    deadband_entry.setDefault(0.02);

    left_pose_sub.setDefault(0);
    right_pose_sub.setDefault(0.32);

    alignment_table
        .getSubTable("left")
        .getDoubleTopic("Reef Target")
        .setPersistent(true);
    alignment_table
        .getSubTable("right")
        .getDoubleTopic("Reef Target")
        .setPersistent(true);
  }

  public static double getMaxOffset() {
    return max_offset_entry.get();
  }

  public static double getDeltaX(ReefSide side) {
    switch (side) {
      case RIGHT:
        return right_delta_x_entry.get();
      case LEFT:
      default:
        return left_delta_x_entry.get();
    }
  }

  public static double getLeftReefTargetPose() {
    return left_pose_sub.get();
  }

  public static double getRightReefTargetPose() {
    return right_pose_sub.get();
  }

  public static double getReefTargetPose(ReefSide side) {
    switch (side) {
      case RIGHT:
        return getRightReefTargetPose();
      case LEFT:
      default:
        return getLeftReefTargetPose();
    }
  }

  private static void setSideDelta(ReefSide side, double value) {
    switch (side) {
      case RIGHT:
        right_delta_x_entry.set(value);
        break;
      case LEFT:
        left_delta_x_entry.set(value);
        break;
    }
  }

  private static void setSideAligned(ReefSide side, boolean aligned) {
    switch (side) {
      case RIGHT:
        right_aligned_entry.set(aligned);
        break;
      case LEFT:
        left_aligned_entry.set(aligned);
        break;
    }
  }

  public static boolean getSideAligned(ReefSide side) {
    switch (side) {
      case RIGHT:
        return right_aligned_entry.get();
      case LEFT:
      default:
        return left_aligned_entry.get();
    }
  }

  @Override
  public void periodic() {
    if (piece_sub.get()) {
      double distSensorOffset = Constants.CoralIntake.mapCoral(distance_sensor_sub.get());

      double robotPoseX = LimelightUtil.getRobotPoseX();

      double maxOffset = max_offset_entry.get();

      boolean aligned = false;
      for (ReefSide side : ReefSide.values()) {
        double targetPoseX = getReefTargetPose(side);

        double deltaX = MathUtil.clamp(
            robotPoseX - targetPoseX + distSensorOffset,
            -maxOffset,
            maxOffset);

        boolean sideAligned = Math.abs(deltaX) < deadband_entry.get();
        aligned = sideAligned || aligned;
        setSideDelta(side, deltaX);
        setSideAligned(side, sideAligned);
      }

      if (piece_sub.get()) {
        aligned_entry.set(aligned);
      } else {
        aligned_entry.set(false);
      }
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AutoDestinations.ReefSide;
import frc.robot.util.LimelightUtil;
import cowlib.Util;

public class AlignmentSystem extends SubsystemBase {
  private static NetworkTable smart_dashboard = NetworkTableInstance.getDefault()
      .getTable("SmartDashboard");
  private static NetworkTable alignment_table = NetworkTableInstance.getDefault()
      .getTable("alignment");

  private static BooleanPublisher aligned_pub = alignment_table
      .getBooleanTopic("Aligned").publish();
  public static BooleanSubscriber aligned_sub = alignment_table
      .getBooleanTopic("Aligned").subscribe(false);
  private static BooleanPublisher right_aligned_pub = alignment_table
      .getSubTable("right").getBooleanTopic("Aligned").publish();
  public static BooleanSubscriber right_aligned_sub = alignment_table
      .getSubTable("right").getBooleanTopic("Aligned").subscribe(false);
  private static BooleanPublisher left_aligned_pub = alignment_table
      .getSubTable("left").getBooleanTopic("Aligned").publish();
  public static BooleanSubscriber left_aligned_sub = alignment_table
      .getSubTable("left").getBooleanTopic("Aligned").subscribe(false);
  private static BooleanSubscriber piece_sub = smart_dashboard
      .getBooleanTopic("Piece Acquired").subscribe(false);
  private static DoubleSubscriber distance_sensor_sub = smart_dashboard
      .getDoubleTopic("Distance Sensor").subscribe(Constants.CoralIntake.distSensorLow);

  private static DoubleSubscriber left_pose_sub = alignment_table
      .getDoubleTopic("Left Reef Target").subscribe(0);
  private static DoubleSubscriber right_pose_sub = alignment_table
      .getDoubleTopic("Right Reef Target").subscribe(0.32);

  private static DoubleSubscriber max_offset_sub = alignment_table
      .getDoubleTopic("Max Offset").subscribe(1);
  private static DoubleSubscriber deadband_sub = alignment_table
      .getDoubleTopic("Deadband").subscribe(0.02, PubSubOption.sendAll(true));

  private static DoublePublisher left_delta_x_pub = alignment_table
      .getSubTable("deltas").getDoubleTopic("Left Delta X").publish();
  private static DoublePublisher right_delta_x_pub = alignment_table
      .getSubTable("deltas").getDoubleTopic("Right Delta X").publish();

  private static DoubleSubscriber left_delta_x_sub = alignment_table
      .getSubTable("deltas").getDoubleTopic("Left Delta X").subscribe(0);
  private static DoubleSubscriber right_delta_x_sub = alignment_table
      .getSubTable("deltas").getDoubleTopic("Right Delta X").subscribe(0);

  /** Creates a new CANdleSystem. */
  public AlignmentSystem() {

  }

  public static double getMaxOffset() {
    return max_offset_sub.get();
  }

  public static double getDeltaX(ReefSide side) {
    switch (side) {
      case RIGHT:
        return right_delta_x_sub.get();
      case LEFT:
      default:
        return left_delta_x_sub.get();
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
        right_delta_x_pub.set(value);
        break;
      case LEFT:
        left_delta_x_pub.set(value);
        break;
    }
  }

  private static void setSideAligned(ReefSide side, boolean aligned) {
    switch (side) {
      case RIGHT:
        right_aligned_pub.set(aligned);
        break;
      case LEFT:
        left_aligned_pub.set(aligned);
        break;
    }
  }

  public static boolean getSideAligned(ReefSide side) {
    switch (side) {
      case RIGHT:
        return right_aligned_sub.get();
      case LEFT:
      default:
        return left_aligned_sub.get();
    }
  }

  @Override
  public void periodic() {
    if (piece_sub.get()) {
      double distSensorOffset = Constants.CoralIntake.mapCoral(distance_sensor_sub.get());

      double robotPoseX = LimelightUtil.getRobotPoseX();

      double maxOffset = max_offset_sub.get();

      boolean aligned = false;
      for (ReefSide side : ReefSide.values()) {
        double targetPoseX = getReefTargetPose(side);

        double deltaX = MathUtil.clamp(
            robotPoseX - targetPoseX + distSensorOffset,
            -maxOffset,
            maxOffset);

        boolean sideAligned = Math.abs(deltaX) < deadband_sub.get();
        aligned = sideAligned || aligned;
        setSideDelta(side, deltaX);
        setSideAligned(side, sideAligned);
      }

      if (piece_sub.get()) {
        aligned_pub.set(aligned);
      } else {
        aligned_pub.set(false);
      }
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashSet;
import java.util.stream.IntStream;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** Add your docs here. */
public class AutoDestinations {
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  // @formatter:off
  private static final HashSet<Integer> reefIDs = new HashSet<>() {{
    // InStream.range takes a range [x,y)
    // Red Reef IDs
    IntStream.range(6, 11 + 1).forEach(id -> add(id));
    // Blue Reed IDs
    IntStream.range(17, 22 + 1).forEach(id -> add(id));
  }};
  // @formatter:on

  private static final double sensorLowerBound = 1.4;
  private static final double sensorUpperBound = 15.7;
  private static final double offsetLowerBound = 0;
  // 0.451 ~half robot width in meters
  private static final double offsetUpperBound = 0.451;

  public enum ReefSide {
    LEFT,
    RIGHT
  }

  // 0.451 ~half robot width in meters
  private static final Transform2d reefRightTransform = new Transform2d(
      // X offset
      // positive is toward the tag
      0,
      // Y offset
      // positive is right when facing the tag
      -0.451,
      Rotation2d.kZero);

  // 0.451 ~half robot width in meters
  private static final Transform2d robotOffset = new Transform2d(
      // X offset
      // normal axis of the tag
      0.451,
      // Y offset
      // positive is left when facing in the direction of the fiducial's normal
      0,
      Rotation2d.k180deg // flip robot to face the tag
  );

  private AutoDestinations() {
  }

  private static Transform2d calculateSensorTransform(double sensorValue) {
    return new Transform2d(
        0,
        cowlib.Util.mapDouble(sensorValue,
            sensorLowerBound, sensorUpperBound,
            offsetLowerBound, offsetUpperBound),
        Rotation2d.kZero);
  }

  private static Pose2d tagPosition(int tagID) {
    // TODO: figure out if orElseThrow is the behaviour we want
    return fieldLayout
        .getTagPose(tagID)
        .map(Pose3d::toPose2d)
        .orElseThrow();
  }

  private static Transform2d sideOffset(ReefSide side) {
    switch (side) {
      case RIGHT:
        return reefRightTransform;
      default:
        return Transform2d.kZero;
    }
  }

  public static Pose2d destinationPose(int tagID, ReefSide side, double sensorValue) {
    return tagPosition(tagID)
        .plus(robotOffset)
        .plus(sideOffset(side))
        .plus(calculateSensorTransform(sensorValue));
  }
}

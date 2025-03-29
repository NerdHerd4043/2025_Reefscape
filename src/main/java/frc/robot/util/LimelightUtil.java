// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class LimelightUtil {

  // The Subscriber "subscribes" to a piece of information, allowing the
  // information to be recieved and updated. More classic example in `Drivebase`.
  // Source:
  // https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html#subscribing-to-a-topic
  public static final DoubleArraySubscriber R_limelightRobotPose = NetworkTableInstance.getDefault()
      .getTable("limelight-right")
      .getDoubleArrayTopic("botpose_targetspace")
      .subscribe(new double[6]);

  public static final DoubleArraySubscriber L_limelightRobotPose = NetworkTableInstance.getDefault()
      .getTable("limelight-left")
      .getDoubleArrayTopic("botpose_targetspace")
      .subscribe(new double[6]);

  // Source:
  // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#basic-targeting-data
  public static String validLimelight() {

    double rightLimelightValid = NetworkTableInstance.getDefault()
        .getTable("limelight-right").getEntry("tv").getDouble(0);
    double leftLimelightValid = NetworkTableInstance.getDefault()
        .getTable("limelight-left").getEntry("tv").getDouble(0);

    if (leftLimelightValid == 1 /* && smallAngleDelta() == 1 */) {
      return "limelight-left";
    }
    if (rightLimelightValid == 1 /* && smallAngleDelta() == 2 */) {
      return "limelight-right";
    } else {
      return "none";
    }
  }

  public static double getRobotPoseX() {

    // This double array is used later to hold information we get from the
    // subscriber. Limelight documentation (as of now) doesn't use a subscriber, but
    // our subscriber is getting the same values that the `botpose_targetspace`
    // gets, and those values are best stored in a double array. Source:
    // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
    double[] R_LimelightRobotPoseArray = new double[6];
    double[] L_LimelightRobotPoseArray = new double[6];

    R_LimelightRobotPoseArray = R_limelightRobotPose.get();
    L_LimelightRobotPoseArray = L_limelightRobotPose.get();

    String validLimelightName = validLimelight();
    if (validLimelightName == "limelight-right" && R_LimelightRobotPoseArray.length > 0) {
      return R_LimelightRobotPoseArray[0];
    }
    if (validLimelightName == "limelight-left" && L_LimelightRobotPoseArray.length > 0) {
      return L_LimelightRobotPoseArray[0];
    } else {
      return 0;
    }
  }

  public static int getLimelightID() {
    RawFiducial[] rightFiducials = LimelightHelpers.getRawFiducials("limelight-right");
    int rightID = 0;
    for (RawFiducial fiducial : rightFiducials) {
      rightID = fiducial.id;
    }

    RawFiducial[] leftFiducials = LimelightHelpers.getRawFiducials("limelight-left");
    int leftID = 0;
    for (RawFiducial fiducial : leftFiducials) {
      leftID = fiducial.id;
    }

    if (rightID > 0) {
      return rightID;
    }
    if (leftID > 0) {
      return leftID;
    } else {
      return 0;
    }
  }

  // FIXME: Test
  // Constructs the robot's field location as a Pose2d using information from the
  // aquired limelight position array.
  public static Pose2d getRobotFieldPose2D(AHRS gyro) {
    double[] r_ll_array = R_limelightRobotPose.get();
    double[] l_ll_array = L_limelightRobotPose.get();
    if (validLimelight() == "limelight-right" && r_ll_array.length > 0) {
      double[] limelightArray = r_ll_array;
      Pose2d pose = new Pose2d(
          limelightArray[0], // x
          limelightArray[1], // y
          gyro.getRotation2d()); // Yaw
      return pose;
    }
    if (validLimelight() == "limelight-left" && l_ll_array.length > 0) {
      double[] limelightArray = l_ll_array;
      Pose2d pose = new Pose2d(
          limelightArray[0], // x
          limelightArray[1], // y
          gyro.getRotation2d()); // Yaw
      return pose;
    } else {
      double[] limelightArray = new double[6];
      Pose2d pose = new Pose2d(
          limelightArray[0], // x
          limelightArray[1], // y
          gyro.getRotation2d()); // Yaw
      return pose;
    }
  }

  // public static double getLimelightLatency() {
  // if (validLimelight() == "limelight-right") {
  // double[] limelightArray = R_limelightRobotPose.get();
  // return limelightArray[5];
  // }

  // if (validLimelight() == "limelight-left") {
  // double[] limelightArray = L_limelightRobotPose.get();
  // return limelightArray[5];
  // }

  // return 0;
  // }

  public static Optional<Double> smallAngleDelta() {
    double[] r_ll_array = R_limelightRobotPose.get();
    double[] l_ll_array = L_limelightRobotPose.get();

    if (r_ll_array.length >= 5 && l_ll_array.length >= 5) {
      SmartDashboard.putNumber("Left LL Angle", l_ll_array[4]);
      SmartDashboard.putNumber("Right LL Angle", r_ll_array[4]);
      return Optional.of(l_ll_array[4] - r_ll_array[4]);
    } else {
      return Optional.empty();
    }

    // if (L_limelightRobotPose.get()[4] < 8) {
    // return 1;
    // }
    // if (R_limelightRobotPose.get()[4] < 8) {
    // return 2;
    // } else {
    // return 0;
    // }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightUtil;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final IntegerPublisher lllThrottleEntry = NetworkTableInstance
      .getDefault()
      .getTable("limelight-left")
      .getIntegerTopic("throttle_set")
      .publish();

  private final IntegerPublisher rllThrottleEntry = NetworkTableInstance
      .getDefault()
      .getTable("limelight-right")
      .getIntegerTopic("throttle_set")
      .publish();

  public Robot() {
    Epilogue.bind(this);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    m_robotContainer = new RobotContainer();

    FollowPathCommand.warmupCommand().schedule();

    lllThrottleEntry.set(200);
    rllThrottleEntry.set(200);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("R Target", LimelightUtil.getLimelightID());

    // SmartDashboard.putNumber("LL Latency", LimelightUtil.getLimelightLatency());
    // // Latency

    // These Smart Dashboard values are used by the CANdleSystem.java subsystem
    Optional<Double> optionalY = LimelightUtil.getYDist();
    optionalY.ifPresent(y -> SmartDashboard.putNumber("LL Y Dist", Math.abs(optionalY.get())));

    var llAngleDelta = LimelightUtil.smallAngleDelta();
    llAngleDelta.ifPresentOrElse(
        angleDelta -> {
          SmartDashboard.putNumber("LL Angle Delta", angleDelta);
          SmartDashboard.putBoolean("Valid LL Angle Delta", true);
        },
        () -> SmartDashboard.putBoolean("Valid LL Angle Delta", false));
  }

  @Override
  public void disabledInit() {
    // Sets the Limelights' detection software to not be looking for anything, so as
    // to save battery.
    // LimelightHelpers.setPipelineIndex("limelight-left", 1);
    // LimelightHelpers.setPipelineIndex("limelight-right", 1);

    lllThrottleEntry.set(200);
    rllThrottleEntry.set(200);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    lllThrottleEntry.set(0);
    rllThrottleEntry.set(0);

    // Sets the Limelights' detection software to looking for April Tags.
    // LimelightHelpers.setPipelineIndex("limelight-left", 0);
    // LimelightHelpers.setPipelineIndex("limelight-right", 0);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.resetCoralPID().schedule();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // This Smart Dashboard value is used by the CANdleSystem.java subsystem
    SmartDashboard.putBoolean("Running Autonomous", true);
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    lllThrottleEntry.set(0);
    rllThrottleEntry.set(0);

    // Sets the Limelights' detection software to looking for April Tags.
    // LimelightHelpers.setPipelineIndex("limelight-left", 0);
    // LimelightHelpers.setPipelineIndex("limelight-right", 0);

    m_robotContainer.resetCoralPID().schedule();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // This Smart Dashboard value is used by the CANdleSystem.java subsystem
    SmartDashboard.putBoolean("Running Autonomous", false);
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANdleConstants;
import frc.robot.util.LimelightUtil;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class CANdleSystem extends SubsystemBase {
  private final CANdle candle = new CANdle(Constants.CANdleConstants.CANdleID, "rio");

  public enum AnimationType {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Flash
  }

  int r;
  int g;
  int b;

  private AnimationType currentAnimation;
  private Animation toAnimate;

  /** Creates a new CANdleSystem. */
  public CANdleSystem() {
    CANdleConfiguration config = new CANdleConfiguration();

    config.disableWhenLOS = true;
    config.stripType = LEDStripType.GRB;
    this.configBrightness(80);
    config.vBatOutputMode = VBatOutputMode.On;

    this.candle.configAllSettings(config);

  }

  public AnimationType getCurrentAnimation() {
    return this.currentAnimation;
  }

  public void setColors(int r, int g, int b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }

  public void setOrange() {
    this.setColors(255, 25, 0);
    this.changeAnimation(AnimationType.SetAll);
  }

  public void setBlue() {
    this.setColors(0, 0, 255);
    this.changeAnimation(AnimationType.SetAll);
  }

  public void setGreen() {
    this.setColors(0, 255, 0);
    this.changeAnimation(AnimationType.SetAll);
  }

  public void setPurple() {
    this.setColors(238, 130, 238);
    this.changeAnimation(AnimationType.SetAll);
  }

  public void setFlashing() {
    this.changeAnimation(AnimationType.Flash);
  }

  public void configBrightness(double percent) {
    this.candle.configBrightnessScalar(percent, 0);
  }

  public void setRainbow() {
    this.changeAnimation(AnimationType.Rainbow);
  }

  public void changeAnimation(AnimationType toChange) {
    this.currentAnimation = toChange;

    if (this.currentAnimation != null) {
      switch (this.currentAnimation) {
        case ColorFlow:
          this.toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, CANdleConstants.ledCount, Direction.Forward);
          this.toAnimate.setLedOffset(8);
          break;
        case Flash:
          this.toAnimate = new StrobeAnimation(238, 130, 238);
          break;
        case Rainbow:
          this.toAnimate = new RainbowAnimation(80, 0.5, CANdleConstants.ledCount);
        default:
          this.toAnimate = null;
          break;
      }
    }
  }

  public void ledsOff() {
    this.setColors(0, 0, 0);
    this.changeAnimation(null);
  }

  public boolean validLimelight() {
    switch (LimelightUtil.validLimelight()) {
      case "limelight-left":
      case "limelight-right":
        return true;
      default:
        return false;
    }
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("Running Autonomous", true)) {
      this.setRainbow();
    }
    if (SmartDashboard.getBoolean("Piece Aquired", false)) {
      this.setOrange();
    }
    if (SmartDashboard.getBoolean("Aligned", true)) {
      this.setGreen();
    }
    if (this.validLimelight()) {
      this.setPurple();
    }
    if (SmartDashboard.getBoolean("Aligning", true)) {
      this.setFlashing();
    } else {
      this.setBlue();
    }

    // Animates the LEDs periodically
    if (this.toAnimate != null) {
      candle.animate(this.toAnimate);
    } else {
      this.candle.setLEDs(r, g, b);
    }
  }
}

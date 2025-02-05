// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotState;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.vision.AlignmentState;
import frc.robot.vision.LimelightLocalization;

public class LED extends SubsystemBase {
  private final AddressableLED m_led;
  private final  RobotManager robotManager;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Timer blinkTimer = new Timer();
  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;
  private LEDState state = new LEDState(Color.kBlue, Patterns.SOLID);

  public LED(RobotManager robotManager) {
    this.robotManager = robotManager;
    blinkTimer.start();
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  @Override
  public void periodic() {
    if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP_CORAL_STATION_ALIGN){
      switch (LimelightLocalization.getInstance().getCoralStationAlignmentState()) {
        case ALIGNED:
        LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
        break;
      case FAR_LEFT:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
      case FAR_RIGHT:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
      case INVALID:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      }
        }
    else if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP_REEF_ALIGN && !RobotManager.getInstance().isHeightCapped){
      switch (LimelightLocalization.getInstance().getReefAlignmentState()) {
        case ALIGNED:
        LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
        break;
      case FAR_LEFT:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
      case FAR_RIGHT:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
        case INVALID:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      }
    }

    else{
      state = robotManager.getState().ledState;
      //m_scrollingRainbo.applyTo(m_ledBuffer);
      switch (state.patterns()) {
        case SOLID:
            LEDPattern.solid(state.color()).applyTo(m_ledBuffer);
          break;
        case SLOW_BLINK:
          if (blinkTimer.get() % (SLOW_BLINK_DURATION * 2) < SLOW_BLINK_DURATION) {
            LEDPattern.solid(state.color()).applyTo(m_ledBuffer);
          } else {
            LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer);
          }
          break;
        case FAST_BLINK:
          if (blinkTimer.get() % (FAST_BLINK_DURATION * 2) < FAST_BLINK_DURATION) {
            LEDPattern.solid(state.color()).applyTo(m_ledBuffer);
          } else {
            LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer);
          }
          break;
        default:
          break;
      }
    }
    m_led.setData(m_ledBuffer);
  }
}
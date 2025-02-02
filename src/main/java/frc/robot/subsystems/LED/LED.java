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

public class LED extends SubsystemBase {
  private static final LEDState FLASH_LIGHTS = new LEDState(Color.kBlue, Patterns.FAST_BLINK);
  private final AddressableLED m_led;
  private final  RobotManager robotManager;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Timer blinkTimer = new Timer();
  private Patterns patterns;
  private Color color;
  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;
  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
 // private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private LEDState state = new LEDState(Color.kBlue, Patterns.SOLID);
  //private final LEDPattern m_scrollingRainbow =
    //.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  /** Called once at the beginning of the robot program. */
  
  
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
    RobotState robotState = robotManager.getState();
    state = robotState.ledState;
    //m_scrollingRainbo.applyTo(m_ledBuffer);
    switch (state.patterns()) {
      case SOLID:
          LEDPattern.solid(state.color()).applyTo(m_ledBuffer);
        break;
      default:
        break;
    }
    m_led.setData(m_ledBuffer);

    double time = blinkTimer.get();
    double onDuration = 0;
    double offDuration = 0;

    if(state.patterns() == Patterns.FAST_BLINK){
      onDuration = FAST_BLINK_DURATION;
      offDuration = FAST_BLINK_DURATION * 2;
    } else if(state.patterns() == Patterns.SLOW_BLINK){
      onDuration = SLOW_BLINK_DURATION;
      offDuration = SLOW_BLINK_DURATION * 2;
    }
  }
}
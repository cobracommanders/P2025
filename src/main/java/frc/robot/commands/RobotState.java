package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED.LedState;
import frc.robot.subsystems.LED.Patterns;

public enum RobotState {
  PREPARE_IDLE(false, false, new LedState(Color.kYellow, Patterns.SOLID)),
  IDLE(false, false, new LedState(Color.kYellow, Patterns.SOLID)),
  PREPARE_INVERTED_IDLE(false, true, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  INVERTED_IDLE(false, true, new LedState(Color.kPurple, Patterns.SOLID)),
  PREPARE_DEEP_CLIMB(true, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  WAIT_DEEP_CLIMB(true, false, new LedState(Color.kGreen, Patterns.FAST_BLINK)),
  DEEP_CLIMB(true, false, new LedState(Color.kCadetBlue, Patterns.SOLID)),
  PREPARE_L1(false, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  WAIT_L1(false, false, new LedState(Color.kGreen, Patterns.FAST_BLINK)),
  SCORE_L1(false, false, new LedState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L2(false, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  WAIT_L2(false, false, new LedState(Color.kGreen, Patterns.FAST_BLINK)),
  SCORE_L2(false, false, new LedState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L3(false, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  WAIT_L3(false, false, new LedState(Color.kGreen, Patterns.FAST_BLINK)),
  SCORE_L3(false, false, new LedState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L4(false, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  WAIT_L4(false, false, new LedState(Color.kGreen, Patterns.FAST_BLINK)),
  SCORE_L4(false, false, new LedState(Color.kGreen, Patterns.SOLID)),
  PREPARE_CAPPED_L4(false, false, new LedState(Color.kDarkBlue, Patterns.SLOW_BLINK)),
  PREPARE_CORAL_STATION(false, false, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  INTAKE_CORAL_STATION(false, false, new LedState(Color.kCoral, Patterns.FAST_BLINK)),
  PREPARE_INVERTED_CORAL_STATION(false, true, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  INVERTED_INTAKE_CORAL_STATION(false, true, new LedState(Color.kCadetBlue, Patterns.SOLID)),
  REMOVE_ALGAE(false, false, new LedState(Color.kCadetBlue, Patterns.SOLID)),
  DRIVE(false, false, new LedState(Color.kAliceBlue, Patterns.SOLID)),
  REEF_ALIGN(false, false, new LedState(Color.kAliceBlue, Patterns.SOLID)),
  CORAL_STATION_ALIGN(false, false, new LedState(Color.kAliceBlue, Patterns.SOLID)),
  PREPARE_INVERTED_FROM_IDLE(true, true, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  PREPARE_IDLE_FROM_INVERTED(true, true, new LedState(Color.kOrangeRed, Patterns.FAST_BLINK)),
  HOMING_STAGE_1_ELEVATOR(true, true, new LedState(Color.kRed, Patterns.SLOW_BLINK)),
  HOMING_STAGE_2_ELBOW(true, true, new LedState(Color.kRed, Patterns.SLOW_BLINK)),
  HOMING_STAGE_3_WRIST(true, true, new LedState(Color.kRed, Patterns.SLOW_BLINK));
  
  public final boolean ignoreRequests;
  public final boolean inverted;
  public final LedState ledState;

  RobotState(boolean ignoreRequests, boolean inverted, LedState ledState){
    this.ignoreRequests = ignoreRequests;
    this.inverted = inverted;
    this.ledState = ledState;
  }

}

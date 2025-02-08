package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.LED.Patterns;

public enum RobotState {
  PREPARE_IDLE(false, false, new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_IDLE(false, false,  new LEDState(Color.kPurple, Patterns.SOLID)),
  IDLE(false, false,  new LEDState(Color.kPurple, Patterns.SOLID)),
  PREPARE_INVERTED_IDLE(false, true,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_INVERTED_IDLE(false, true,  new LEDState(Color.kYellow, Patterns.SOLID)),
  INVERTED_IDLE(false, true,  new LEDState(Color.kYellow, Patterns.SOLID)),
  PREPARE_DEEP_CLIMB(true, false,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_DEEP_CLIMB(true, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  DEEP_CLIMB(true, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L1(false, true,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_L1(false, true,  new LEDState(Color.kGreen, Patterns.SOLID)),
  SCORE_L1(false, true,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L2(false, false,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_L2(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  SCORE_L2(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L3(false, false,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_L3(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  SCORE_L3(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L4(false, false,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  WAIT_L4(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  L4_ELBOW(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  SCORE_L4(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  CAPPED_L4(false, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  CAPPED_L3(false, false, new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  PREPARE_CORAL_STATION(false, false,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  INTAKE_CORAL_STATION(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_INVERTED_CORAL_STATION(false, true,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  INVERTED_INTAKE_CORAL_STATION(false, true,  new LEDState(Color.kGreen, Patterns.SOLID)),
  REMOVE_ALGAE(false, false,  new LEDState(Color.kPurple, Patterns.SOLID)),
  PREPARE_INVERTED_FROM_IDLE(true, true,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  PREPARE_IDLE_FROM_INVERTED(true, true,  new LEDState(Color.kOrangeRed, Patterns.SOLID)),
  HOMING_STAGE_1_ELEVATOR(true, true, new LEDState(Color.kRed, Patterns.SOLID)),
  HOMING_STAGE_2_ELBOW(true, true, new LEDState(Color.kRed, Patterns.SOLID)),
  HOMING_STAGE_3_WRIST(true, true,  new LEDState(Color.kRed, Patterns.SOLID)),
  PREPARE_HOMING(true, true, new LEDState(Color.kRed, Patterns.SOLID));

  public final boolean ignoreRequests;
  public final boolean inverted;
  public final LEDState ledState;

  RobotState(boolean ignoreRequests, boolean inverted, LEDState ledState){
    this.ignoreRequests = ignoreRequests;
    this.inverted = inverted;
    this.ledState = ledState;
  }

}

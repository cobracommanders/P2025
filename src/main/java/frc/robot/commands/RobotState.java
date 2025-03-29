package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.LED.Patterns;

public enum RobotState {
  PREPARE_IDLE(false, false, new LEDState(Color.kPurple)),
  WAIT_IDLE(false, false,  new LEDState(Color.kBlue)),
  IDLE(false, false,  new LEDState(Color.kBlue)),
  PREPARE_INVERTED_IDLE(false, true,  new LEDState(Color.kPurple)),
  INVERTED_IDLE(false, true,  new LEDState(Color.kBlue)),
  DEEP_CLIMB_DEPLOY(true, false,  new LEDState(Color.kYellow)),
  DEEP_CLIMB_RETRACT(true, false,  new LEDState(Color.kYellowGreen)),
  DEEP_CLIMB_UNWIND(true, false,  new LEDState(Color.kYellowGreen)),
  DEEP_CLIMB_WAIT(true, false, new LEDState (Color.kGreen)),
  PREPARE_DEEP_CLIMB(true, false, new LEDState(Color.kYellowGreen)),
  PREPARE_L1(false, true,  new LEDState(Color.kPurple)),
  WAIT_L1(false, true,  new LEDState(Color.kYellowGreen)),
  SCORE_L1(false, true,  new LEDState(Color.kGreen)),
  PREPARE_L2(false, false,  new LEDState(Color.kPurple)),
  WAIT_L2(false, false,  new LEDState(Color.kYellowGreen)),
  SCORE_L2(false, false,  new LEDState(Color.kGreen)),
  PREPARE_L3(false, false,  new LEDState(Color.kPurple)),
  WAIT_L3(false, false,  new LEDState(Color.kYellowGreen)),
  SCORE_L3(false, false,  new LEDState(Color.kGreen)),
  PREPARE_L4(false, false,  new LEDState(Color.kPurple)),
  WAIT_L4(false, false,  new LEDState(Color.kYellowGreen)),
  L4_ELBOW(false, false,  new LEDState(Color.kGreen)),
  PRE_L4(false, false,  new LEDState(Color.kGreen)),
  PRE_HEIGHT_L4(false, false,  new LEDState(Color.kGreen)),
  SCORE_L4(false, false,  new LEDState(Color.kGreen)),
  CAPPED_L4(false, false,  new LEDState(Color.kYellowGreen)),
  PREPARE_CORAL_STATION(false, false,  new LEDState(Color.kPurple)),
  INTAKE_CORAL_STATION(false, false,  new LEDState(Color.kWhite)),
  PREPARE_INVERTED_CORAL_STATION(false, true,  new LEDState(Color.kPurple)),
  INVERTED_INTAKE_CORAL_STATION(false, true,  new LEDState(Color.kWhite)),
  POST_INVERTED_CORAL_STATION_INTAKE(false, true,  new LEDState(Color.kWhite)),
  PREPARE_INVERTED_FROM_IDLE(true, true,  new LEDState(Color.kPurple)),
  PREPARE_IDLE_FROM_INVERTED(true, true,  new LEDState(Color.kPurple)),
  HOMING_STAGE_1_ELEVATOR(false, true, new LEDState(Color.kRed)),
  HOMING_STAGE_2_ELBOW(true, true, new LEDState(Color.kRed)),
  HOMING_STAGE_3_WRIST(true, true,  new LEDState(Color.kRed)),
  PREPARE_HOMING(true, true, new LEDState(Color.kRed)),
  PREPARE_REMOVE_ALGAE_LOW(true, false, new LEDState(Color.kBlue)),
  WAIT_REMOVE_ALGAE_LOW(false, false, new LEDState(Color.kBlue)),
  REMOVE_ALGAE_LOW(false, false, new LEDState(Color.kBlue)),
  PREPARE_PROCESSOR(true, false, new LEDState(Color.kBlue)),
  WAIT_PROCESSOR(false, false, new LEDState(Color.kBlue)),
  SCORE_PROCESSOR(false, false, new LEDState(Color.kBlue)),
  PREPARE_REMOVE_ALGAE_HIGH(true, false, new LEDState(Color.kBlue)),
  WAIT_REMOVE_ALGAE_HIGH(false, false, new LEDState(Color.kBlue)),
  REMOVE_ALGAE_HIGH(false, false, new LEDState(Color.kBlue)),
  PREPARE_SCORE_ALGAE(false, false, new LEDState(Color.kBlue)),
  SCORE_ALGAE_WAIT(false, false, new LEDState(Color.kBlue)),
  SCORE_ALGAE(false, false, new LEDState(Color.kBlue));


  public final boolean ignoreRequests;
  public final boolean inverted;
  public final LEDState ledState;

  RobotState(boolean ignoreRequests, boolean inverted, LEDState ledState){
    this.ignoreRequests = ignoreRequests;
    this.inverted = inverted;
    this.ledState = ledState;
  }
}

package frc.robot.commands;

public enum RobotState {
  PREPARE_IDLE(false),
  WAIT_IDLE(false),
  IDLE(false),
  PREPARE_INVERTED_IDLE(false),
  WAIT_INVERTED_IDLE(false),
  INVERTED_IDLE(false),
  PREPARE_DEEP_CLIMB(true),
  WAIT_DEEP_CLIMB(true),
  DEEP_CLIMB(true),
  PREPARE_L1(false),
  WAIT_L1(false),
  SCORE_L1(false),
  PREPARE_L2(false),
  WAIT_L2(false),
  SCORE_L2(false),
  PREPARE_L3(false),
  WAIT_L3(false),
  SCORE_L3(false),
  PREPARE_L4(false),
  WAIT_L4(false),
  SCORE_L4(false),
  PREPARE_CAPPED_L4(false),
  WAIT_CAPPED_L4(false),
  PREPARE_CORAL_STATION(false),
  WAIT_CORAL_STATION(false),
  CORAL_STATION(false),
  PREPARE_INVERTED_CORAL_STATION(false),
  WAIT_INVERTED_CORAL_STATION(false),
  INVERTED_CORAL_STATION(false);

  public final boolean climbing;

  RobotState(boolean climbing){
    this.climbing = climbing;
  }

}

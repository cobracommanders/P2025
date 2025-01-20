package frc.robot.commands;

public enum RobotState {
  PREPARE_IDLE(false, false),
  WAIT_IDLE(false, false),
  IDLE(false, false),
  HOME_ELEVATOR(false, false),
  PREPARE_INVERTED_IDLE(false, true),
  WAIT_INVERTED_IDLE(false, true),
  INVERTED_IDLE(false, true),
  PREPARE_DEEP_CLIMB(true, false),
  WAIT_DEEP_CLIMB(true, false),
  DEEP_CLIMB(true, false),
  PREPARE_L1(false, false),
  WAIT_L1(false, false),
  SCORE_L1(false, false),
  PREPARE_L2(false, false),
  WAIT_L2(false, false),
  SCORE_L2(false, false),
  PREPARE_L3(false, false),
  WAIT_L3(false, false),
  SCORE_L3(false, false),
  PREPARE_L4(false, false),
  WAIT_L4(false, false),
  SCORE_L4(false, false),
  PREPARE_CAPPED_L4(false, false),
  WAIT_CAPPED_L4(false, false),
  PREPARE_CORAL_STATION(false, false),
  INTAKE_CORAL_STATION(false, false),
  PREPARE_INVERTED_CORAL_STATION(false, true),
  INVERTED_INTAKE_CORAL_STATION(false, true),
  PREPARE_INVERTED_FROM_IDLE(true, true),
  PREPARE_IDLE_FROM_INVERTED(true, true);

  public final boolean ignoreRequests;
  public final boolean inverted;

  RobotState(boolean climbing, boolean inverted){
    this.ignoreRequests = climbing;
    this.inverted = inverted;
  }

}

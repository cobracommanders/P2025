package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.StateMachine;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorState;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.FlagManager;

public class RobotManager extends StateMachine<RobotState> {
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final ManipulatorSubsystem manipulator;
  public final WristSubsystem wrist;
  public final ElbowSubsystem elbow;
  public final CommandSwerveDrivetrain drivetrain;

  private RobotState state = RobotState.IDLE;

  public boolean isHeightCapped = false;
  public Timer timer = new Timer();

  private final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

  public RobotManager() {
    super(RobotState.IDLE);
    this.elevator = ElevatorSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.manipulator = ManipulatorSubsystem.getInstance();
    this.wrist = WristSubsystem.getInstance();
    this.elbow = ElbowSubsystem.getInstance();
    this.drivetrain = CommandSwerveDrivetrain.getInstance();
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    RobotState nextState = currentState;
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case APPLY_HEIGHT_CAP: 
          isHeightCapped = true;
          break;
        case REMOVE_HEIGHT_CAP:
          isHeightCapped = false;
          break;
        case IDLE:
          if (!currentState.ignoreRequests) {
            nextState = (!currentState.inverted) ? RobotState.PREPARE_IDLE : RobotState.PREPARE_IDLE_FROM_INVERTED;
          }
          break;
        case INVERTED_IDLE:
          if (!currentState.ignoreRequests) {
            nextState = (currentState.inverted) ? RobotState.PREPARE_INVERTED_IDLE : RobotState.PREPARE_INVERTED_FROM_IDLE;
          }
          break;
        case CORAL_STATION:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_CORAL_STATION;
          }
          break;
        case INVERTED_CORAL_STATION:
          if (!currentState.ignoreRequests && currentState.inverted) {
            nextState = RobotState.PREPARE_INVERTED_CORAL_STATION;
          }
          break;
        case L1:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L1;
          }
          break;
        case L2:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L2;
          }
          break;
        case L3:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L3;
          }
          break;
        case L4:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = isHeightCapped ? RobotState.PREPARE_CAPPED_L4 : RobotState.PREPARE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (currentState == RobotState.IDLE) {
            nextState = RobotState.DEEP_CLIMB;
          }
          break;
        case SCORE:
          switch (nextState) {
            case WAIT_L1:
              nextState = RobotState.SCORE_L1;
              break;
            case WAIT_L2:
              nextState = RobotState.SCORE_L2;
              break;
            case WAIT_L3:
             nextState = RobotState.SCORE_L3;
              break;
            case WAIT_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case DEEP_CLIMB:
            default:
              break;
          }
      }
    }
    //automatic transitions
    switch (state) {
      case WAIT_L2:
      case IDLE:
      case WAIT_INVERTED_IDLE:
      case WAIT_DEEP_CLIMB:
      case WAIT_CAPPED_L4:
      case WAIT_IDLE:
      case WAIT_L1:
      case DEEP_CLIMB:
      case WAIT_L3:
      case INVERTED_INTAKE_CORAL_STATION:
      case INTAKE_CORAL_STATION:
      case INVERTED_IDLE:
        break;
      

      case PREPARE_L1:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L1;
        }
        break;
      case PREPARE_L2:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L2;
        }
        break;
      case PREPARE_L3:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L3;
        }
        break;
      case PREPARE_CAPPED_L4:
        if (isHeightCapped == false) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L4:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L4;
        }
        break;
      case WAIT_L4:
        if(isHeightCapped == true) {
          nextState = RobotState.PREPARE_CAPPED_L4;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INVERTED_INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.IDLE;
        }
        break;
      case PREPARE_INVERTED_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L1:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L2:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L3:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L4:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case PREPARE_INVERTED_FROM_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_IDLE_FROM_INVERTED:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_IDLE_FROM_INVERTED;
        }
        break;
      case PREPARE_DEEP_CLIMB:
        if (climber.atGoal()) {
          nextState = RobotState.WAIT_DEEP_CLIMB;
        }
        break;
    }
    flags.clear();
    return nextState;
  };
  

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
          case IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }
          case PREPARE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case SCORE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case PREPARE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);      
          }
          case SCORE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
          }
          case PREPARE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case SCORE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case PREPARE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.L4);
            elbow.setState(ElbowState.L4);
          }
          case SCORE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L4);
            wrist.setState(WristState.L4);
            elbow.setState(ElbowState.L4);
          }
          case PREPARE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case PREPARE_INVERTED_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case INVERTED_INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);      
          }
          case PREPARE_DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.PREPARE_DEEP_CLIMB);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }
          case DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }
          case INVERTED_IDLE,
            PREPARE_CAPPED_L4,
            PREPARE_IDLE,
            PREPARE_IDLE_FROM_INVERTED,
            PREPARE_INVERTED_FROM_IDLE,
            PREPARE_INVERTED_IDLE, 
            WAIT_CAPPED_L4,
            WAIT_DEEP_CLIMB, 
            WAIT_IDLE, 
            WAIT_INVERTED_IDLE, 
            WAIT_L1, 
            WAIT_L2, 
            WAIT_L3, 
            WAIT_L4 -> {}
          }
      }

  @Override
  public void periodic() {
    super.periodic(); 
  }

  public void idleRequest() {
    flags.check(RobotFlag.IDLE);
  }

  public void prepareL1Request() {
    flags.check(RobotFlag.L1);
  }

  public void prepareL2Request() {
    flags.check(RobotFlag.L2);
  }

  public void prepareL3Request() {
    flags.check(RobotFlag.L3);
  }
  
  public void prepareL4Request() {
    flags.check(RobotFlag.L4);
  }
  
  public void prepareDeepClimbRequest() {
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void prepareCoralStationRequest() {
    flags.check(RobotFlag.CORAL_STATION);
  }

  public void prepareInvertedCoralStationRequest() {
    flags.check(RobotFlag.INVERTED_CORAL_STATION);
  }

  public void scoreRequest() {
    flags.check(RobotFlag.SCORE);
  }




  public void stopScoringRequest() {
    switch (getState()) {
      case DEEP_CLIMB-> {}
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
  private static RobotManager instance;
  
  public static RobotManager getInstance() {
    if (instance == null) instance = new RobotManager(); // Make sure there is an instance (this will only run once)
    return instance;
}

}
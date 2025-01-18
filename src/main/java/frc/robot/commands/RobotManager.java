package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.RobotFlag;

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
        case APPLY_HEIGHT_CAP: isHeightCapped = true;
          break;
        case REMOVE_HEIGHT_CAP:
          isHeightCapped = false;
          break;
        case CORAL_STATION:
          if (!currentState.climbing) {
            nextState = RobotState.PREPARE_CORAL_STATION;
          }
          break;
        case INVERTED_CORAL_STATION:
          if (!currentState.climbing) {
            nextState = RobotState.PREPARE_INVERTED_CORAL_STATION;
          }
        break;
          case L1:
          if (!currentState.climbing) {
            nextState = RobotState.PREPARE_L1;
          }
          break;
        case L2:
          if (!currentState.climbing) {
            nextState = RobotState.PREPARE_L2;
          }
          break;
        case L3:
          if (!currentState.climbing) {
            nextState = RobotState.PREPARE_L3;
          }
          break;
        case L4:
          if (!currentState.climbing) {
            nextState = isHeightCapped ? RobotState.PREPARE_CAPPED_L4 : RobotState.PREPARE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (!currentState.climbing) {
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
      case WAIT_L4:
      case WAIT_CORAL_STATION:
      case WAIT_CAPPED_L4:
      case WAIT_IDLE:
      case WAIT_L1:
      case PREPARE_DEEP_CLIMB:
      case DEEP_CLIMB:
      case WAIT_L3:
      case INVERTED_CORAL_STATION:
      case WAIT_INVERTED_CORAL_STATION:
      case CORAL_STATION:
      case INVERTED_IDLE:

      

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
        if (isHeightCapped = false) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L4:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L4;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_INVERTED_CORAL_STATION;
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
        if (timeout(0)) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L2:
        if (timeout(0)) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L3:
        if (timeout(0)) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L4:
        if (timeout(0)) {
          nextState = RobotState.INVERTED_IDLE;
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
      }
      case SCORE_L1 -> {
        elevator.setState(ElevatorState.L2);
      }
      case PREPARE_L2 -> {
        elevator.setState(ElevatorState.L3);
      }
      case SCORE_L2 -> {
        elevator.setState(ElevatorState.L4);
      }
      case PREPARE_L3 -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case SCORE_L3 -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case PREPARE_L4 -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case SCORE_L4 -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case INVERTED_CORAL_STATION -> {
        elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
      }
      case CORAL_STATION -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case PREPARE_CORAL_STATION -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case PREPARE_INVERTED_CORAL_STATION -> {
        elevator.setState(ElevatorState.CORAL_STATION);
      }
      case DEEP_CLIMB -> {
        elevator.setState(ElevatorState.IDLE);
      }
      
      }
    }

  @Override
  public void periodic() {
    super.periodic(); 
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case APPLY_HEIGHT_CAP:
          isHeightCapped = true;
          break;
        case REMOVE_HEIGHT_CAP:
          isHeightCapped = false;
          break;
        case CORAL_STATION:
          if (!state.climbing) {
            state = RobotState.PREPARE_CORAL_STATION;
          }
          break;
        case INVERTED_CORAL_STATION:
          if (!state.climbing) {
            state = RobotState.PREPARE_INVERTED_CORAL_STATION;
          }
        break;
          case L1:
          if (!state.climbing) {
            state = RobotState.PREPARE_L1;
          }
          break;
        case L2:
          if (!state.climbing) {
            state = RobotState.PREPARE_L2;
          }
          break;
        case L3:
          if (!state.climbing) {
            state = RobotState.PREPARE_L3;
          }
          break;
        case L4:
          if (!state.climbing) {
            state = isHeightCapped ? RobotState.PREPARE_CAPPED_L4 : RobotState.PREPARE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (!state.climbing) {
            state = RobotState.DEEP_CLIMB;
          }
          break;
        case SCORE:
          switch (state) {
            case WAIT_L1:
              state = RobotState.SCORE_L1;
              break;
            case WAIT_L2:
              state = RobotState.SCORE_L2;
              break;
            case WAIT_L3:
              state = RobotState.SCORE_L3;
              break;
            case WAIT_L4:
              state = RobotState.SCORE_L4;
              break;
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
      case WAIT_L4:
      case WAIT_CORAL_STATION:
      case WAIT_CAPPED_L4:
      case WAIT_IDLE:
      case WAIT_L1:
      case PREPARE_DEEP_CLIMB:
      case DEEP_CLIMB:
      case WAIT_L3:
      case INVERTED_CORAL_STATION:
      case WAIT_INVERTED_CORAL_STATION:
      case CORAL_STATION:
      case INVERTED_IDLE:

      

      case PREPARE_L1:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_L1;
        }
        break;
      case PREPARE_L2:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_L2;
        }
        break;
      case PREPARE_L3:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_L3;
        }
        break;
      case PREPARE_CAPPED_L4:
        if (isHeightCapped = false) {
          state = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L4:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_L4;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.WAIT_INVERTED_CORAL_STATION;
        }
        break;
      case PREPARE_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.IDLE;
        }
        break;
      case PREPARE_INVERTED_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          state = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L1:
        if (timeout(1)) {
          state = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L2:
        if (timeout(1)) {
          state = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L3:
        if (timeout(1)) {
          state = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L4:
        if (timeout(1)) {
          state = RobotState.INVERTED_IDLE;
        }
        break;
    }

    flags.clear();
  }

  public void idleRequest() {
    setStateFromRequest(RobotState.IDLE);
  }

  public void scoreRequest() {
    setStateFromRequest(RobotState.SCORE_L1);
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
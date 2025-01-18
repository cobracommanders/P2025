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

  private final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

  public RobotManager(
      ElevatorSubsystem elevator,
      ClimberSubsystem climber,
      ManipulatorSubsystem manipulator,
      WristSubsystem wrist,
      ElbowSubsystem elbow,
      CommandSwerveDrivetrain drivetrain) {
    super(RobotState.IDLE);
    this.elevator = elevator;
    this.climber = climber;
    this.manipulator = manipulator;
    this.wrist = wrist;
    this.elbow = elbow;
    this.drivetrain = drivetrain;
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case 
      PREPARE_IDLE,
      WAIT_IDLE,
      IDLE,
      PREPARE_INVERTED_IDLE,
      WAIT_INVERTED_IDLE,
      INVERTED_IDLE,
      PREPARE_DEEP_CLIMB,
      WAIT_DEEP_CLIMB,
      DEEP_CLIMB,
      PREPARE_L1,
      WAIT_L1,
      L1,
      PREPARE_L2,
      WAIT_L2,
      L2,
      PREPARE_L3,
      WAIT_L3,
      L3,
      PREPARE_L4,
      WAIT_L4,
      L4,
      PREPARE_CAPPED_L4,
      WAIT_CAPPED_L4,
      CAPPED_L4,
      PREPARE_CORAL_STATION,
      WAIT_CORAL_STATION,
      CORAL_STATION,
      PREPARE_INVERTED_CORAL_STATION,
      WAIT_INVERTED_CORAL_STATION,
      INVERTED_CORAL_STATION ->
          currentState;
      };
    };
  

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        elevator.setState(ElevatorState.IDLE);
        climber.setState(ClimberState.IDLE);
        manipulator.setState(ManipulatorState.IDLE);
        wrist.setState(WristState.IDLE);
        
      }
      case L1 -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case L2 -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case L3 -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case L4 -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case CORAL_STATION -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case INVERTED_CORAL_STATION -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case CAPPED_L4 -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      case DEEP_CLIMB -> {
        elevatorSubsystem.setState(ElevatorState.L1);
      }
      
      }
    }

  @Override
  public void periodic() {
    super.periodic(); 
  }

  public void idleRequest() {
    setStateFromRequest(RobotState.IDLE);
  }

  public void scoreRequest() {
    setStateFromRequest(RobotState.L1);
  }


  public void stopScoringRequest() {
    switch (getState()) {
      case DEEP_CLIMB-> {}
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
}
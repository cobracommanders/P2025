package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.StateMachine;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final ElevatorSubsystem elevatorSubsystem;

  public RobotManager(
      ElevatorSubsystem elevatorSubsystem) {
    super(RobotState.IDLE);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE,
            DEEP_CLIMB,
            L1,
            L2,
            L3,
            L4 ->
          currentState;
      };
    };
  

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        elevatorSubsystem.setState(ElevatorState.IDLE);
      }
      case L1 -> {
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
      case L1-> {}

      case L2-> {}

      case L3-> {}

      case L4-> {}

      case DEEP_CLIMB-> {}

      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
}
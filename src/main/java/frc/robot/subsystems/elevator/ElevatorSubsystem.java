package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
    
  private final TalonFX motor;
  private double elevatorPosition;
  
  private final PIDController pidController;
  
  private ElevatorState currentState;
  private double setpoint;
  private double GEAR_RATIO = 224.0/16200.0;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public ElevatorSubsystem() {
    super(ElevatorState.IDLE);
    motor = new TalonFX(Ports.ElevatorPorts.ELEVATOR_MOTOR);
    currentState = ElevatorState.IDLE;
    pidController = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);
  }

   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(ElevatorPositions.IDLE, elevatorPosition, 0.1);
      case L1 ->
        MathUtil.isNear(ElevatorPositions.L1, elevatorPosition, 0.1);
      case L2 ->
        MathUtil.isNear(ElevatorPositions.L2, elevatorPosition, 0.1);
      case L3 ->
        MathUtil.isNear(ElevatorPositions.L3, elevatorPosition, 0.1);
      case L4 ->
        MathUtil.isNear(pidController.set(ElevatorPositions.L4), elevatorPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, 0.1);
    };
  }

  public void setState(ElevatorState newState) {
      setStateFromRequest(newState);
    }

  public void getPosition(){
    elevatorPosition = motor.getPosition().getValueAsDouble();
  }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          motor.set(ElevatorPositions.IDLE);
        }
        case L1 -> {
          motor.set(ElevatorPositions.L1);
        }
        case L2 -> {
          motor.set(ElevatorPositions.L2);
        }
        case L3 -> {
          motor.set(ElevatorPositions.L3);
        }
        case L4 -> {
          motor.set(ElevatorPositions.L4);
        }
        case CORAL_STATION -> {
          motor.set(ElevatorPositions.CORAL_STATION);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
  }

  public void set(double speed) {
      motor.set(speed);
  }

  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
      if (instance == null) instance = new ElevatorSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
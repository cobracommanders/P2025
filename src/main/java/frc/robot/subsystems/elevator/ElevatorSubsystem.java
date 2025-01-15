package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
    
  private final TalonFX motor;
  
  private ElevatorState currentState;
  private double setpoint;
  private double GEAR_RATIO = 224.0/16200.0;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public ElevatorSubsystem() {
      super(ElevatorState.IDLE);
      // motor = new LazySparkMax(Ports.IntakePorts.LMOTOR, MotorType.kBrushless);
      motor = new TalonFX(Ports.ElevatorPorts.ELEVATOR_MOTOR);
      
      currentState = ElevatorState.IDLE;
  }

  public void setState(ElevatorState newState) {
      setStateFromRequest(newState);
    }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          motor.set(0.0);
        }
        case L1 -> {
          motor.set(0);
        }
        case L2 -> {
          motor.set(0);
        }
        case L3 -> {
          motor.set(0);
        }
        case L4 -> {
          motor.set(0);
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
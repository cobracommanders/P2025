package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState>{
    
  private final TalonFX lMotor;
  private final TalonFX rMotor;
  
  private ClimberState currentState;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
  private double GEAR_RATIO = 224.0/16200.0;
  
  public ClimberSubsystem() {
      super(ClimberState.IDLE);
      // motor = new LazySparkMax(Ports.IntakePorts.LMOTOR, MotorType.kBrushless);
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      lMotor = new TalonFX(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR);
      rMotor = new TalonFX(Ports.ClimberPorts.RIGHT_CLIMBER_MOTOR);  
      lMotor.getConfigurator().apply(motor_config);
      rMotor.getConfigurator().apply(motor_config);    
      
      currentState = ClimberState.IDLE;
  }

  public void setState(ClimberState newState) {
      setStateFromRequest(newState);
    }

    @Override
    protected void afterTransition(ClimberState newState) {
      switch (newState) {
        case IDLE -> {
          lMotor.set(0.0);
          rMotor.set(0.0);
        }
        case DEEP_CLIMB_WAIT -> {
          lMotor.set(0.0);
          rMotor.set(0.0);
        }
        case DEEP_CLIMB_RETRACT -> {
          lMotor.set(1);
          rMotor.set(-1);
        }
        case DEEP_CLIMB_DEPLOY -> {
          lMotor.set(-1);
          rMotor.set(1);
        }
        case DEEP_CLIMB_UNWIND -> {
          lMotor.set(-1);
          rMotor.set(1);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {

  }

  public boolean atGoal(){
    return true;
  }

  public void set(double speed) {
      lMotor.set(speed);
      rMotor.set(speed);
  }

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
      if (instance == null) instance = new ClimberSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
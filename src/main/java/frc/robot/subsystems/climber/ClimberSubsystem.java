package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.commands.RobotFlag;

public class ClimberSubsystem extends StateMachine<ClimberState>{
    
  private final TalonFX motor;
  
  private ClimberState currentState;
  private double setpoint;
  private double GEAR_RATIO = 224.0/16200.0;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public ClimberSubsystem() {
      super(ClimberState.IDLE);
      // motor = new LazySparkMax(Ports.IntakePorts.LMOTOR, MotorType.kBrushless);
      motor = new TalonFX(Ports.ClimberPorts.CLIMBER_MOTOR);
      
      
      currentState = ClimberState.IDLE;
  }

  public void setState(ClimberState newState) {
      setStateFromRequest(newState);
    }

    @Override
    protected void afterTransition(ClimberState newState) {
      switch (newState) {
        case IDLE -> {
          motor.set(0.0);
        }
        case DEEP_CLIMB -> {
          motor.set(0);
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
      motor.set(speed);
  }

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
      if (instance == null) instance = new ClimberSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Ports;
import frc.robot.StateMachine;



public class KickerSubsystem extends StateMachine<KickerState>{
    private final TalonFX kickerMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private double kickerSpeed;
    
    public KickerSubsystem() {
      super(KickerState.IDLE);
      kickerMotor = new TalonFX(Ports.KickerPorts.KICKER_MOTOR);
      kickerMotor.getConfigurator().apply(motor_config);
    }

    @Override
    public void collectInputs(){
      kickerSpeed = kickerMotor.get();
    }
  
     public boolean atGoal() {
      return switch (getState()) {
        case IDLE -> 
          KickerSpeeds.IDLE == kickerSpeed;
        case REMOVE_ALGAE -> 
          KickerSpeeds.REMOVE_ALGAE == kickerSpeed;
      };
    }
  
    public void setState(KickerState newState) {
        setStateFromRequest(newState);
    }
  
    public void setKickerPositions(double kickerSpeed){
      DogLog.log(getName() + "/Kicker speed", kickerSpeed);
      kickerMotor.set(kickerSpeed);
    }
  
      @Override
      protected void afterTransition(KickerState newState) {
        switch (newState) {
          case IDLE -> {
            setKickerPositions(KickerSpeeds.IDLE);
          }
          case REMOVE_ALGAE -> {
            setKickerPositions(KickerSpeeds.REMOVE_ALGAE);
          }
        }
      }
  
    private static KickerSubsystem instance;
  
    public static KickerSubsystem getInstance() {
        if (instance == null) instance = new KickerSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
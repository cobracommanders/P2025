package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elevator.ElevatorPositions;



public class ManipulatorSubsystem extends StateMachine<ManipulatorState>{
    public final TalonFX manipulatorMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private Timer intakeTimer = new Timer();
    private double manipulatorSpeed;
    private double manipulatorStatorCurrent;
    
    public ManipulatorSubsystem() {
      super(ManipulatorState.IDLE);
      manipulatorMotor = new TalonFX(Ports.ManipulatorPorts.MANIPULATOR_MOTOR);
      motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motor_config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.8;
      motor_config.CurrentLimits.StatorCurrentLimit = 100;
      manipulatorMotor.getConfigurator().apply(motor_config);
    }

    protected ManipulatorState getNextState(ManipulatorState currentState) {
      return currentState;
    }

    @Override
    public void collectInputs(){
      manipulatorSpeed = manipulatorMotor.get();
      manipulatorStatorCurrent = manipulatorMotor.getStatorCurrent().getValueAsDouble();
      DogLog.log(getName() + "/Motor Stator Current", manipulatorStatorCurrent);
    }
  
    public void setState(ManipulatorState newState) {
        setStateFromRequest(newState);
    }

    public boolean hasCoral(){
      if (manipulatorStatorCurrent > Constants.ManipulatorConstants.coralStallCurrent){
        return true;
      } else {
        return false;
      }
    }
  
    public void setManipulatorPositions(double manipulatorSpeed){
      DogLog.log(getName() + "/Manipulator speed", manipulatorSpeed);
      manipulatorMotor.set(manipulatorSpeed);
    }
  
      @Override
      protected void afterTransition(ManipulatorState newState) {
        switch (newState) {
          case IDLE -> {
            setManipulatorPositions(ManipulatorSpeeds.IDLE);
          }
          case INTAKE_CORAL -> {
            setManipulatorPositions(ManipulatorSpeeds.INTAKE_CORAL);
          }
          case AFTER_INTAKE -> {
            setManipulatorPositions(ManipulatorSpeeds.AFTER_INTAKE);
          }
          case L1 -> {
            setManipulatorPositions(ManipulatorSpeeds.L1);
          }
          case L2 -> {
            setManipulatorPositions(ManipulatorSpeeds.L2);
          }
          case L3 -> {
            setManipulatorPositions(ManipulatorSpeeds.L3);
          }
          case L4 -> {
            setManipulatorPositions(ManipulatorSpeeds.L4);
          }
          case PRE_SCORE -> {
            setManipulatorPositions(ManipulatorSpeeds.PRE_SCORE);
          }
          case INTAKE_ALGAE -> {
            setManipulatorPositions(ManipulatorSpeeds.INTAKE_ALGAE);
          }
          case SCORE_ALGAE -> {
            setManipulatorPositions(ManipulatorSpeeds.SCORE_ALGAE);
          }
          case SCORE_PROCESSOR -> {
            setManipulatorPositions(ManipulatorSpeeds.SCORE_PROCESSOR);
          }
          default -> {}
        }
      }
  
    private static ManipulatorSubsystem instance;
  
    public static ManipulatorSubsystem getInstance() {
        if (instance == null) instance = new ManipulatorSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
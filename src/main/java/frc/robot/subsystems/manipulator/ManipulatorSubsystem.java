package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.RobotManager;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.wrist.WristState;



public class ManipulatorSubsystem extends StateMachine<ManipulatorState>{
    public final TalonFX manipulatorMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private double manipulatorSpeed;
    
    public ManipulatorSubsystem() {
      super(ManipulatorState.IDLE);
      manipulatorMotor = new TalonFX(Ports.ManipulatorPorts.MANIPULATOR_MOTOR);
      motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      manipulatorMotor.getConfigurator().apply(motor_config);
    }

    protected ManipulatorState getNextState(ManipulatorState currentState) {
      return currentState;
    }

    @Override
    public void collectInputs(){
      manipulatorSpeed = manipulatorMotor.get();
    }
  
     public boolean atGoal() {
      return true;
      // return switch (getState()) {
      //   case IDLE -> 
      //     ManipulatorSpeeds.IDLE == manipulatorSpeed;
      //   case INTAKE_CORAL -> 
      //     manipulatorMotor.getStatorCurrent().getValueAsDouble() > ManipulatorConstants.coralStallCurrent;
      //   case L1 ->
      //     ManipulatorSpeeds.L1 == manipulatorSpeed;
      //   case L2 ->
      //     ManipulatorSpeeds.L2 == manipulatorSpeed;
      //   case L3 ->
      //     ManipulatorSpeeds.L3 == manipulatorSpeed;
      //   case L4 ->
      //     ManipulatorSpeeds.L4 == manipulatorSpeed;
      //   case PREPARE_L1 ->
      //     ManipulatorSpeeds.PREPARE_L1 == manipulatorSpeed;
      //   case PREPARE_L2 ->
      //     ManipulatorSpeeds.PREPARE_L2 == manipulatorSpeed;
      //   case PREPARE_L3 ->
      //     ManipulatorSpeeds.PREPARE_L3 == manipulatorSpeed;
      //   case PREPARE_L4 ->
      //     ManipulatorSpeeds.PREPARE_L4 == manipulatorSpeed;
      //   case AFTER_INTAKE->
      //     ManipulatorSpeeds.AFTER_INTAKE == manipulatorSpeed;
      };
  
    public void setState(ManipulatorState newState) {
        setStateFromRequest(newState);
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
          default -> {}
        }
      }
  
    private static ManipulatorSubsystem instance;
  
    public static ManipulatorSubsystem getInstance() {
        if (instance == null) instance = new ManipulatorSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
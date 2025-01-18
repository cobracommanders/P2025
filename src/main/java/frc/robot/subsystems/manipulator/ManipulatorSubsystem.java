package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;



public class ManipulatorSubsystem extends StateMachine<ManipulatorState>{
    private final TalonFX manipulatorMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private double manipulatorPosition;
  
    private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
    
    private ManipulatorState currentState;
    private double setpoint;
    private double manualSpeed;
    
    private boolean isActivated = true;
    
    public ManipulatorSubsystem() {
      super(ManipulatorState.IDLE);
      manipulatorMotor = new TalonFX(Ports.ManipulatorPorts.MANIPULATOR_MOTOR);
      currentState = ManipulatorState.IDLE;
    }
  
     public boolean atGoal() {
      return switch (getState()) {
        case IDLE -> 
          MathUtil.isNear(ManipulatorPositions.IDLE, manipulatorPosition, 0.1);
        case INTAKE_CORAL -> 
          MathUtil.isNear(ManipulatorPositions.IDLE, manipulatorPosition, 0.1);
        case L1 ->
          MathUtil.isNear(ManipulatorPositions.L1, manipulatorPosition, 0.1);
        case L2 ->
          MathUtil.isNear(ManipulatorPositions.L2, manipulatorPosition, 0.1);
        case L3 ->
          MathUtil.isNear(ManipulatorPositions.L3, manipulatorPosition, 0.1);
        case L4 ->
          MathUtil.isNear(ManipulatorPositions.L4, manipulatorPosition, 0.1);
      };
    }
  
    public void setState(ManipulatorState newState) {
        setStateFromRequest(newState);
      }
  
    public void updatePosition(){
      manipulatorPosition = manipulatorMotor.getPosition().getValueAsDouble();
    }
  
    public void setManipulatorPositions(double manipulatorPosition){
      manipulatorMotor.setControl(motor_request.withPosition(manipulatorPosition));
    }
  
      @Override
      protected void afterTransition(ManipulatorState newState) {
        switch (newState) {
          case IDLE -> {
            setManipulatorPositions(ManipulatorPositions.IDLE);
          }
          case INTAKE_CORAL -> {
            setManipulatorPositions(ManipulatorPositions.INTAKE_CORAL);
          }
          case L1 -> {
            setManipulatorPositions(ManipulatorPositions.L1);
          }
          case L2 -> {
            setManipulatorPositions(ManipulatorPositions.L2);
          }
          case L3 -> {
            setManipulatorPositions(ManipulatorPositions.L3);
          }
          case L4 -> {
            setManipulatorPositions(ManipulatorPositions.L4);
          }
          default -> {}
        }
      }
  
    @Override
    public void periodic() {
      updatePosition();
    }
    private static ManipulatorSubsystem instance;
  
    public static ManipulatorSubsystem getInstance() {
        if (instance == null) instance = new ManipulatorSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
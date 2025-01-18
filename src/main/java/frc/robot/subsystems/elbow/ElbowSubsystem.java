package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElbowSubsystem extends StateMachine<ElbowState>{
    
  private final TalonFX motor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double elbowPosition;

  private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
  
  private final PIDController pidController;
  
  private ElbowState currentState;
  private double setpoint;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public ElbowSubsystem() {
    super(ElbowState.IDLE);
    motor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
    currentState = ElbowState.IDLE;
    pidController = new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);
  }

   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(ElbowPositions.IDLE, elbowPosition, 0.1);
      case INVERTED_IDLE -> 
        MathUtil.isNear(ElbowPositions.IDLE, elbowPosition, 0.1);
      case L1 ->
        MathUtil.isNear(ElbowPositions.L1, elbowPosition, 0.1);
      case L2 ->
        MathUtil.isNear(ElbowPositions.L2, elbowPosition, 0.1);
      case L3 ->
        MathUtil.isNear(ElbowPositions.L3, elbowPosition, 0.1);
      case CAPPED_L4 ->
        MathUtil.isNear(ElbowPositions.CAPPED_L4, elbowPosition, 0.1);
      case L4 ->
        MathUtil.isNear(ElbowPositions.L3, elbowPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.CORAL_STATION, elbowPosition, 0.1);
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.CORAL_STATION, elbowPosition, 0.1);
    };
  }

  public void setState(ElbowState newState) {
      setStateFromRequest(newState);
    }

  public void updatePosition(){
    elbowPosition = motor.getPosition().getValueAsDouble();
  }

  public void setElbowPosition(double position){
    motor.setControl(motor_request.withPosition(position));
  }

    @Override
    protected void afterTransition(ElbowState newState) {
      switch (newState) {
        case IDLE -> {
          setElbowPosition(ElbowPositions.IDLE);
        }
        case L1 -> {
          setElbowPosition(ElbowPositions.L1);
        }
        case L2 -> {
          setElbowPosition(ElbowPositions.L2);
        }
        case L3 -> {
          setElbowPosition(ElbowPositions.L3);
        }
        case CAPPED_L4 -> {
          setElbowPosition(ElbowPositions.CAPPED_L4);
        }
        case L4 -> {
          setElbowPosition(ElbowPositions.L4);
        }
        case CORAL_STATION -> {
          setElbowPosition(ElbowPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setElbowPosition(ElbowPositions.CORAL_STATION);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
    updatePosition();
  }
  private static ElbowSubsystem instance;

  public static ElbowSubsystem getInstance() {
      if (instance == null) instance = new ElbowSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
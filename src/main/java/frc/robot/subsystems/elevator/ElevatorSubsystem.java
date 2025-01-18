package frc.robot.subsystems.elevator;

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

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
    
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double elevatorPosition;

  private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
  
  private final PIDController pidController;
  
  private ElevatorState currentState;
  private double setpoint;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public ElevatorSubsystem() {
    super(ElevatorState.IDLE);
    leftMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
    rightMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);
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
      case CAPPED_L4 ->
        MathUtil.isNear(ElevatorPositions.CAPPED_L4, elevatorPosition, 0.1);
      case L4 ->
        MathUtil.isNear(ElevatorPositions.L3, elevatorPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, 0.1);
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, 0.1);
    };
  }

  public void setState(ElevatorState newState) {
      setStateFromRequest(newState);
    }

  public void updatePosition(){
    elevatorPosition = leftMotor.getPosition().getValueAsDouble();
  }

  public void setElevatorPosition(double leftPosition, double rightPosition){
    leftMotor.setControl(motor_request.withPosition(leftPosition));
    rightMotor.setControl(motor_request.withPosition(rightPosition));
  }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          setElevatorPosition(ElevatorPositions.IDLE, -ElevatorPositions.IDLE);
        }
        case L1 -> {
          setElevatorPosition(ElevatorPositions.L1, -ElevatorPositions.L1);
        }
        case L2 -> {
          setElevatorPosition(ElevatorPositions.L2, -ElevatorPositions.L2);
        }
        case L3 -> {
          setElevatorPosition(ElevatorPositions.L3, -ElevatorPositions.L3);
        }
        case CAPPED_L4 -> {
          setElevatorPosition(ElevatorPositions.CAPPED_L4, -ElevatorPositions.CAPPED_L4);
        }
        case L4 -> {
          setElevatorPosition(ElevatorPositions.L4, -ElevatorPositions.L4);
        }
        case CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.CORAL_STATION, -ElevatorPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.CORAL_STATION, -ElevatorPositions.CORAL_STATION);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
    updatePosition();
  }
  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
      if (instance == null) instance = new ElevatorSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.WristConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class WristSubsystem extends StateMachine<WristState>{
    
  private final TalonFX wristMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(WristConstants.P).withKI(WristConstants.I).withKD(WristConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double wristPosition;

  private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
  
  private final PIDController pidController;
  
  private WristState currentState;
  private double setpoint;
  private double manualSpeed;
  
  private boolean isActivated = true;
  
  public WristSubsystem() {
    super(WristState.IDLE);
    wristMotor = new TalonFX(Ports.WristPorts.WRIST_MOTOR);
    currentState = WristState.IDLE;
    pidController = new PIDController(WristConstants.P, WristConstants.I, WristConstants.D);
  }

   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(WristPositions.IDLE, wristPosition, 0.1);
      case INVERTED_IDLE -> 
        MathUtil.isNear(WristPositions.INVERTED_IDLE, wristPosition, 0.1);
      case L1 ->
        MathUtil.isNear(WristPositions.L1, wristPosition, 0.1);
      case L2 ->
        MathUtil.isNear(WristPositions.L2, wristPosition, 0.1);
      case L3 ->
        MathUtil.isNear(WristPositions.L3, wristPosition, 0.1);
      case CAPPED_L4 ->
        MathUtil.isNear(WristPositions.CAPPED_L4, wristPosition, 0.1);
      case L4 ->
        MathUtil.isNear(WristPositions.L4, wristPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(WristPositions.CORAL_STATION, wristPosition, 0.1);
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(WristPositions.INVERTED_CORAL_STATION, wristPosition, 0.1);
    };
  }

  public void setState(WristState newState) {
      setStateFromRequest(newState);
    }

  public void updatePosition(){
    wristPosition = wristMotor.getPosition().getValueAsDouble();
  }

  public void setWristPosition(double wristPosition){
    wristMotor.setControl(motor_request.withPosition(wristPosition));
  }

    @Override
    protected void afterTransition(WristState newState) {
      switch (newState) {
        case IDLE -> {
          setWristPosition(WristPositions.IDLE);
        }
        case INVERTED_IDLE -> {
          setWristPosition(WristPositions.INVERTED_IDLE);
        }
        case L1 -> {
          setWristPosition(WristPositions.L1);
        }
        case L2 -> {
          setWristPosition(WristPositions.L2);
        }
        case L3 -> {
          setWristPosition(WristPositions.L3);
        }
        case CAPPED_L4 -> {
          setWristPosition(WristPositions.CAPPED_L4);
        }
        case L4 -> {
          setWristPosition(WristPositions.L4);
        }
        case CORAL_STATION -> {
          setWristPosition(WristPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setWristPosition(WristPositions.INVERTED_CORAL_STATION);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
    updatePosition();
  }
  private static WristSubsystem instance;

  public static WristSubsystem getInstance() {
      if (instance == null) instance = new WristSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
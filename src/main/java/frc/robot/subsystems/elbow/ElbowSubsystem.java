package frc.robot.subsystems.elbow;

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
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.RobotManager;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elevator.ElevatorState;

public class ElbowSubsystem extends StateMachine<ElbowState>{
    
  private final TalonFX motor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElbowConstants.P).withKI(ElbowConstants.I).withKD(ElbowConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double elbowPosition;
  private double motorPosition;
  private String elbowState;

  private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
  
  public ElbowSubsystem() {
    super(ElbowState.IDLE);
    motor = new TalonFX(Ports.ElbowPorts.MOTOR);
    motor.getConfigurator().apply(motor_config);
  }
  
  protected ElbowState getNextState(ElbowState currentState) {
    if (getState() == ElbowState.HOME_ELBOW && this.atGoal()) { 
      motor.setPosition(0);
      return ElbowState.IDLE;
    } else {
      return currentState;
    }
  }

   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(ElbowPositions.IDLE, elbowPosition, 0.1);
      case INVERTED_IDLE -> 
        MathUtil.isNear(ElbowPositions.INVERTED_IDLE, elbowPosition, 0.1);
      case L1 ->
        MathUtil.isNear(ElbowPositions.L1, elbowPosition, 0.1);
      case L2 ->
        MathUtil.isNear(ElbowPositions.L2, elbowPosition, 0.1);
      case L3 ->
        MathUtil.isNear(ElbowPositions.L3, elbowPosition, 0.1);
      case CAPPED_L4 ->
        MathUtil.isNear(ElbowPositions.CAPPED_L4, elbowPosition, 0.1);
      case L4 ->
        MathUtil.isNear(ElbowPositions.L4, elbowPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.CORAL_STATION, elbowPosition, 0.1);
      case HOME_ELBOW ->
        motor.getStatorCurrent().getValueAsDouble() > ElbowConstants.homingStallCurrent;
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.INVERTED_CORAL_STATION, elbowPosition, 0.1);
    };
  }

  public void setState(ElbowState newState) {
      setStateFromRequest(newState);
      DogLog.log(getName() + "/Elbow State", newState);
  }

  @Override
  public void collectInputs() {
    motorPosition = motor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Elbow Position", motorPosition);
  }

  public void setElbowPosition(double position) {
    motor.setControl(motor_request.withPosition(position));
    motorPosition = motor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Elbow Position", motorPosition);
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

  private static ElbowSubsystem instance;

  public static ElbowSubsystem getInstance() {
      if (instance == null) instance = new ElbowSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
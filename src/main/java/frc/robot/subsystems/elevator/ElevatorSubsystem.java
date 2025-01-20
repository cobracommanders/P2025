package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFXConfiguration left_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private final TalonFXConfiguration right_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double elevatorPosition;
  private double leftMotorPosition;
  private double rightMotorPosition;
  private double leftMotorSetpoint;
  private double rightMotorSetpoint;

  private PositionVoltage left_motor_request = new PositionVoltage(0).withSlot(0);
  private PositionVoltage right_motor_request = new PositionVoltage(0).withSlot(0);

  public ElevatorSubsystem() {
    super(ElevatorState.IDLE);
    right_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    left_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
    rightMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);
    leftMotor.getConfigurator().apply(left_motor_config);
    rightMotor.getConfigurator().apply(right_motor_config);
  }

  protected ElevatorState getNextState(ElevatorState currentState) {
    if (this.atGoal()) {
      return ElevatorState.IDLE;
    } else {
      return currentState;
    }
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
        MathUtil.isNear(ElevatorPositions.L4, elevatorPosition, 0.1);
      case CORAL_STATION ->
        MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, 0.1);
      case HOME_ELEVATOR ->
        leftMotor.getStatorCurrent().getValueAsDouble() > ElevatorConstants.homingStallCurrent;
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(ElevatorPositions.INVERTED_CORAL_STATION, elevatorPosition, 0.1);
    };
  }

  public void setState(ElevatorState newState) {
      setStateFromRequest(newState);
    }

  @Override
  public void collectInputs(){
    elevatorPosition = leftMotor.getPosition().getValueAsDouble();
    leftMotorPosition = leftMotor.getPosition().getValueAsDouble();
    rightMotorPosition = rightMotor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Position", elevatorPosition);
    DogLog.log(getName() + "/Left Motor Position", leftMotorPosition);
    DogLog.log(getName() + "/Right Motor Position", rightMotorPosition);
  }

  public void setElevatorPosition(double leftPosition, double rightPosition){
    leftMotor.setControl(left_motor_request.withPosition(leftPosition));
    rightMotor.setControl(right_motor_request.withPosition(rightPosition));
    DogLog.log(getName() + "/Left Motor Setpoint", leftPosition);
    DogLog.log(getName() + "/Right Motor Setpoint", rightPosition);
  }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          setElevatorPosition(ElevatorPositions.IDLE, ElevatorPositions.IDLE);
        }
        case L1 -> {
          setElevatorPosition(ElevatorPositions.L1, ElevatorPositions.L1);
        }
        case L2 -> {
          setElevatorPosition(ElevatorPositions.L2, ElevatorPositions.L2);
        }
        case L3 -> {
          setElevatorPosition(ElevatorPositions.L3, ElevatorPositions.L3);
        }
        case CAPPED_L4 -> {
          setElevatorPosition(ElevatorPositions.CAPPED_L4, ElevatorPositions.CAPPED_L4);
        }
        case L4 -> {
          setElevatorPosition(ElevatorPositions.L4, ElevatorPositions.L4);
        }
        case HOME_ELEVATOR -> {
          leftMotor.set(-0.02);
          rightMotor.set(-0.02);
        }
        case CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.CORAL_STATION, ElevatorPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.INVERTED_CORAL_STATION, ElevatorPositions.INVERTED_CORAL_STATION);
        }
        default -> {}
      }
    }
    // @Override
    // public void periodic() {
    //   super.periodic();
      
    // }
  
  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
      if (instance == null) instance = new ElevatorSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
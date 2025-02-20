package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFXConfiguration left_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D).withKG(ElevatorConstants.G).withGravityType(GravityTypeValue.Elevator_Static)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private final TalonFXConfiguration right_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D).withKG(ElevatorConstants.G).withGravityType(GravityTypeValue.Elevator_Static)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private double elevatorPosition;
  private double leftElevatorPosition;
  private double leftMotorPosition;
  private double rightMotorPosition;
  private final double tolerance;
  private Follower right_motor_request = new Follower(Ports.ElevatorPorts.LMOTOR, true);
  private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = Double.POSITIVE_INFINITY;

  public ElevatorSubsystem() {
    super(ElevatorState.HOME_ELEVATOR);
    right_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    left_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
    left_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
    left_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
    right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
    right_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
    right_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
    leftMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
    rightMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);
    leftMotor.getConfigurator().apply(left_motor_config);
    rightMotor.getConfigurator().apply(right_motor_config);
    tolerance = 0.1;
  }

  protected ElevatorState getNextState(ElevatorState currentState) {
    if (getState() == ElevatorState.HOME_ELEVATOR && this.atGoal()) { 
      rightMotor.setPosition(0);
      leftMotor.setPosition(0);
      return ElevatorState.IDLE;
    } else {
      return currentState;
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
        case IDLE -> 
          MathUtil.isNear(ElevatorPositions.IDLE, elevatorPosition, tolerance);
        case L1 ->
          MathUtil.isNear(ElevatorPositions.L1, elevatorPosition, tolerance);
        case L2 ->
          MathUtil.isNear(ElevatorPositions.L2, elevatorPosition, tolerance);
        case L3 ->
          MathUtil.isNear(ElevatorPositions.L3, elevatorPosition, tolerance);
        case CAPPED_L3 ->
          MathUtil.isNear(ElevatorPositions.CAPPED_L3, elevatorPosition, tolerance);
        case CAPPED_L4 ->
          MathUtil.isNear(ElevatorPositions.CAPPED_L4, elevatorPosition, tolerance);
        case L4 ->
          MathUtil.isNear(ElevatorPositions.L4, elevatorPosition, tolerance);
        case CORAL_STATION ->
          MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, tolerance);
        case HOME_ELEVATOR ->
          (rightMotor.getStatorCurrent().getValueAsDouble() > ElevatorConstants.homingStallCurrent);
        case INVERTED_CORAL_STATION ->
          MathUtil.isNear(ElevatorPositions.INVERTED_CORAL_STATION, elevatorPosition, tolerance);
      };
  }

  public void setState(ElevatorState newState) {
      setStateFromRequest(newState);
    }

  public boolean isIdle() {
    return getState() == ElevatorState.IDLE;
  }

  @Override
  public void collectInputs(){
    elevatorPosition = leftMotor.getPosition().getValueAsDouble();
    double leftElevatorPosition = elevatorPosition;
    double rightElevatorPosition = rightMotor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Left Elevator Position", leftElevatorPosition);
    DogLog.log(getName() + "/Right Elevator Position", rightElevatorPosition);
    DogLog.log(getName() + "/Elevator Current", leftMotor.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void periodic(){
    super.periodic();
    DogLog.log(getName() + "/Elevator AtGoal", atGoal());
    // if (DriverStation.isDisabled()) {
    //  if (lowestSeenHeight > elevatorPosition) {
    //   lowestSeenHeight = elevatorPosition;
    //  }
    // } else {
    //   if (!preMatchHomingOccured) {
    //     double homingEndPosition = 0;
    //     double homedPosition = homingEndPosition + (elevatorPosition - lowestSeenHeight);
    //     rightMotor.setPosition(homedPosition);
    //     preMatchHomingOccured = true;
    //   }
    // }
  }

  public void setElevatorPosition(double elevatorPosition){
    rightMotor.setControl(right_motor_request);
    leftMotor.setControl(left_motor_request.withPosition(elevatorPosition));
    //DogLog.log(getName() + "/Left Motor Setpoint", leftMotorPosition);
    DogLog.log(getName() + "/right Motor Setpoint", elevatorPosition);
  }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          setElevatorPosition(ElevatorPositions.IDLE);
        }
        case L1 -> {
          setElevatorPosition(ElevatorPositions.L1);
        }
        case L2 -> {
          setElevatorPosition(ElevatorPositions.L2);
        }
        case L3 -> {
          setElevatorPosition(ElevatorPositions.L3);
        }
        case CAPPED_L3 -> {
          setElevatorPosition(ElevatorPositions.CAPPED_L3);
        }
        case CAPPED_L4 -> {
          setElevatorPosition(ElevatorPositions.CAPPED_L4);
        }
        case L4 -> {
          setElevatorPosition(ElevatorPositions.L4);
        }
        case HOME_ELEVATOR -> {
          rightMotor.setControl(new VoltageOut(-0.7));
        }
        case CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.INVERTED_CORAL_STATION);
        }
      }
    }
  
  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
      if (instance == null) instance = new ElevatorSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
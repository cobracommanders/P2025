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
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElbowConstants.P).withKI(ElbowConstants.I).withKD(ElbowConstants.D).withKG(ElbowConstants.G)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((54.0179 / 1.0)));
  private double elbowPosition;
  private final double tolerance;

  private PositionVoltage motor_request = new PositionVoltage(0).withSlot(0);
  
  public ElbowSubsystem() {
    super(ElbowState.IDLE);
    motor = new TalonFX(Ports.ElbowPorts.MOTOR);
    motor.getConfigurator().apply(motor_config);
    tolerance = 0.2;
  }
  
  protected ElbowState getNextState(ElbowState currentState) {
    if (getState() == ElbowState.HOME_ELBOW && this.atGoal()) { 
      motor.setPosition(0);
      return ElbowState.INVERTED_IDLE;
    } else {
      return currentState;
    }
  }

   public boolean atGoal() {
    return true;
    // return switch (getState()) {
    //   case IDLE -> 
    //     MathUtil.isNear(ElbowPositions.IDLE, elbowPosition, tolerance);
    //   case INVERTED_IDLE -> 
    //     MathUtil.isNear(ElbowPositions.INVERTED_IDLE, elbowPosition, tolerance);
    //   case L1 ->
    //     MathUtil.isNear(ElbowPositions.L1, elbowPosition, tolerance);
    //   case L2 ->
    //     MathUtil.isNear(ElbowPositions.L2, elbowPosition, tolerance);
    //   case L3 ->
    //     MathUtil.isNear(ElbowPositions.L3, elbowPosition, tolerance);
    //   case CAPPED_L4 ->
    //     MathUtil.isNear(ElbowPositions.CAPPED_L4, elbowPosition, tolerance);
    //   case L4 ->
    //     MathUtil.isNear(ElbowPositions.L4, elbowPosition, tolerance);
    //   case CORAL_STATION ->
    //     MathUtil.isNear(ElbowPositions.CORAL_STATION, elbowPosition, tolerance);
    //   case HOME_ELBOW ->
    //     motor.getStatorCurrent().getValueAsDouble() > ElbowConstants.homingStallCurrent;
    //   case INVERTED_CORAL_STATION ->
    //     MathUtil.isNear(ElbowPositions.INVERTED_CORAL_STATION, elbowPosition, tolerance);
    // };
  }

  public void setState(ElbowState newState) {
      setStateFromRequest(newState);
      DogLog.log(getName() + "/Elbow State", newState);
  }

  @Override
  public void collectInputs() {
    elbowPosition = motor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Elbow Position", elbowPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    DogLog.log(getName() + "/Elbow AtGoal", atGoal());
  }

  public void setElbowPosition(double position) {
    motor.setControl(motor_request.withPosition(position));
    DogLog.log(getName() + "/Elbow Setpoint", position);
  }

    @Override
    protected void afterTransition(ElbowState newState) {
      switch (newState) {
        case IDLE -> {
          setElbowPosition(ElbowPositions.IDLE);
        }
        case INVERTED_IDLE -> {
          setElbowPosition(ElbowPositions.INVERTED_IDLE);
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
          setElbowPosition(ElbowPositions.INVERTED_CORAL_STATION);
        }
        case HOME_ELBOW -> {
          motor.set(-0.02);
        }
      }
    }

  private static ElbowSubsystem instance;

  public static ElbowSubsystem getInstance() {
      if (instance == null) instance = new ElbowSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
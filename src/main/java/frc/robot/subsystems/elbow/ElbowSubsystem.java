package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElbowSubsystem extends StateMachine<ElbowState>{
    
  private final TalonFX motor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElbowConstants.P).withKI(ElbowConstants.I).withKD(ElbowConstants.D).withKG(ElbowConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((52.381 / 1.0)));
  private double elbowPosition;
  private final double tolerance;
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = Double.POSITIVE_INFINITY;
  private boolean brakeModeEnabled;
  private double motorCurrent;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public ElbowSubsystem() {
    super(ElbowState.HOME_ELBOW);
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor = new TalonFX(Ports.ElbowPorts.MOTOR);
    motor.getConfigurator().apply(motor_config);
    motor_config.MotionMagic.MotionMagicCruiseVelocity = ElbowConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = ElbowConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = ElbowConstants.MotionMagicJerk;
    tolerance = 0.04;
    brakeModeEnabled = false;
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
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(ElbowPositions.IDLE, elbowPosition, tolerance);
      case INVERTED_IDLE -> 
        MathUtil.isNear(ElbowPositions.INVERTED_IDLE, elbowPosition, tolerance);
      case L1 ->
        MathUtil.isNear(ElbowPositions.L1, elbowPosition, tolerance);
      case L2 ->
        MathUtil.isNear(ElbowPositions.L2, elbowPosition, tolerance);
      case L3 ->
        MathUtil.isNear(ElbowPositions.L3, elbowPosition, tolerance);
      case CAPPED_L4 ->
        MathUtil.isNear(ElbowPositions.CAPPED_L4, elbowPosition, tolerance);
      case L4 ->
        MathUtil.isNear(ElbowPositions.L4, elbowPosition, tolerance);
      case HIGH_ALGAE ->
        MathUtil.isNear(ElbowPositions.HIGH_ALGAE, elbowPosition, tolerance);
      case LOW_ALGAE ->
        MathUtil.isNear(ElbowPositions.LOW_ALGAE, elbowPosition, tolerance);
      case PRE_SCORE_ALGAE ->
        MathUtil.isNear(ElbowPositions.PRE_SCORE_ALGAE, elbowPosition, tolerance);
      case CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.CORAL_STATION, elbowPosition, tolerance);
      case HOME_ELBOW ->
        motorCurrent > ElbowConstants.homingStallCurrent;
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(ElbowPositions.INVERTED_CORAL_STATION, elbowPosition, tolerance);
      case L4_ELBOW ->
        MathUtil.isNear(ElbowPositions.L4_ELBOW, elbowPosition, tolerance);
      case CAGE_FLIP ->
        MathUtil.isNear(ElbowPositions.CAGE_FLIP, elbowPosition, tolerance);
      case PROCESSOR ->
        MathUtil.isNear(ElbowPositions.PROCESSOR, elbowPosition, tolerance);
      case DISABLED ->
        true;
     };
  }

  public void setState(ElbowState newState) {
      setStateFromRequest(newState);
  }

  public boolean isIdle() {
    return getState() == ElbowState.INVERTED_IDLE;
  }

  @Override
  public void collectInputs() {
    elbowPosition = motor.getPosition().getValueAsDouble();
    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    DogLog.log(getName() + "/Elbow Position", elbowPosition);
    DogLog.log(getName() + "/Elbow current", motorCurrent);
    DogLog.log(getName() + "/Elbow AtGoal", atGoal());
  }

  @Override
  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled() && brakeModeEnabled == true) {
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      motor.getConfigurator().apply(motor_config);
      brakeModeEnabled = false;
      }
    else if (DriverStation.isEnabled() && brakeModeEnabled == false)  {
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motor.getConfigurator().apply(motor_config);
      brakeModeEnabled = true;
    }

     if (DriverStation.isDisabled()) {
      if (lowestSeenHeight > elbowPosition) {
        lowestSeenHeight = elbowPosition;
        } else {

        if (!preMatchHomingOccured) {
            double homingEndPosition = 0;
            double homedPosition = homingEndPosition + (elbowPosition - lowestSeenHeight);
            motor.setPosition(homedPosition);

            preMatchHomingOccured = true;
          }
        }
      }
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
        case HIGH_ALGAE -> {
          setElbowPosition(ElbowPositions.HIGH_ALGAE);
        }
        case LOW_ALGAE -> {
          setElbowPosition(ElbowPositions.LOW_ALGAE);
        }
        case PRE_SCORE_ALGAE -> {
          setElbowPosition(ElbowPositions.PRE_SCORE_ALGAE);
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
        case L4_ELBOW -> {
          setElbowPosition(ElbowPositions.L4_ELBOW);
        }
        case HOME_ELBOW -> {
          motor.setControl(new VoltageOut(-0.38));
        }
        case CAGE_FLIP -> {
          setElbowPosition(ElbowPositions.CAGE_FLIP);
        }
        case PROCESSOR -> {
          setElbowPosition(ElbowPositions.PROCESSOR);
        }
        case DISABLED -> {
          motor.setControl(new VoltageOut(0));
        }
      }
    }

  private static ElbowSubsystem instance;

  public static ElbowSubsystem getInstance() {
      if (instance == null) instance = new ElbowSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
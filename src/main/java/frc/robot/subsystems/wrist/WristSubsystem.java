package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends StateMachine<WristState>{
    
  private final TalonFX wristMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(WristConstants.P).withKI(WristConstants.I).withKD(WristConstants.D).withKG(Constants.WristConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((18.0 / 1.0)));
  private double wristPosition;
  private final double tolerance;
  private boolean brakeModeEnabled;
  // private boolean preMatchHomingOccured = false;
  // private double lowestSeenHeight = 0.0;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public WristSubsystem() {
    super(WristState.HOME_WRIST);
    wristMotor = new TalonFX(Ports.WristPorts.WRIST_MOTOR);
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor_config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = WristConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = WristConstants.MotionMagicJerk;
    wristMotor.getConfigurator().apply(motor_config);
    tolerance = 0.02;
    brakeModeEnabled = false;
  }
  protected WristState getNextState(WristState currentState) {
    if (getState() == WristState.HOME_WRIST && this.atGoal()) { 
      wristMotor.setPosition(0.38);
      return WristState.INVERTED_IDLE;
    } else {
      return currentState;
    }
  }
   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(WristPositions.IDLE, wristPosition, tolerance);
      case INVERTED_IDLE ->
        MathUtil.isNear(WristPositions.INVERTED_IDLE, wristPosition, tolerance);
      case L1 ->
        MathUtil.isNear(WristPositions.L1, wristPosition, tolerance);
      case L2 ->
        MathUtil.isNear(WristPositions.L2, wristPosition, tolerance);
      case L3 ->
        MathUtil.isNear(WristPositions.L3, wristPosition, tolerance);
      case CAPPED_L3 ->
        MathUtil.isNear(WristPositions.CAPPED_L3, wristPosition, tolerance);
      case CAPPED_L4 ->
        MathUtil.isNear(WristPositions.CAPPED_L4, wristPosition, tolerance);
      case L4 ->
        MathUtil.isNear(WristPositions.L4, wristPosition, tolerance);
      case CORAL_STATION ->
        MathUtil.isNear(WristPositions.CORAL_STATION, wristPosition, tolerance);
      case HOME_WRIST ->
        wristMotor.getStatorCurrent().getValueAsDouble() > WristConstants.homingStallCurrent;
      case INVERTED_CORAL_STATION ->
        MathUtil.isNear(WristPositions.INVERTED_CORAL_STATION, wristPosition, tolerance);
      case AFTER_L4 ->
        MathUtil.isNear(WristPositions.AFTER_L4, wristPosition, tolerance);
      case DISABLED->
        true;
    };
  }

  public void setState(WristState newState) {
      setStateFromRequest(newState);
    }

  public boolean isIdle() {
    return getState() == WristState.INVERTED_IDLE;
  }

    @Override
  public void collectInputs(){
    wristPosition = wristMotor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Wrist Position", wristPosition);
  }

  @Override
  public void periodic(){
    super.periodic();
    DogLog.log(getName() + "/Wrist AtGoal", atGoal());
    // DogLog.log(getName() + "/wrist current", wristMotor.getStatorCurrent().getValueAsDouble());

      if (DriverStation.isDisabled() && brakeModeEnabled == true) {
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wristMotor.getConfigurator().apply(motor_config);
        brakeModeEnabled = false;
      }
      else if (DriverStation.isEnabled() && brakeModeEnabled == false)  {
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristMotor.getConfigurator().apply(motor_config);
        brakeModeEnabled = true;
      }
      

    // if (!preMatchHomingOccured) {
    //   double homingEndPosition = 0;
    //   double homedPosition = homingEndPosition + (wristPosition - lowestSeenHeight);
    //   wristMotor.setPosition(homedPosition);

    //   preMatchHomingOccured = true;
    //   }
    // }
  }

  public void setWristPosition(double position){
    wristMotor.setControl(motor_request.withPosition(position));
    DogLog.log(getName() + "/Wrist Setpoint", position);
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
        case HOME_WRIST -> {
          wristMotor.setControl(new VoltageOut(0.38));
        }
        case DISABLED-> {
          wristMotor.setControl(new VoltageOut(0));
        }
        case AFTER_L4 -> {
          setWristPosition(WristPositions.AFTER_L4);
        }
        default -> {}
      }
    }

  private static WristSubsystem instance;

  public static WristSubsystem getInstance() {
      if (instance == null) instance = new WristSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
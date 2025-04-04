package frc.robot.subsystems.wrist;

import javax.sound.sampled.Port;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotMode;
import frc.robot.commands.RobotState;

public class WristSubsystem extends StateMachine<WristState>{
    
  public final TalonFX wristMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(WristConstants.P).withKI(WristConstants.I).withKD(WristConstants.D).withKG(Constants.WristConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((18.0 / 1.0)));
  private double wristPosition;
  private double motorCurrent;
  private final double tolerance;
  private boolean brakeModeEnabled;
  private final DutyCycle encoder;
  private double absolutePosition;
  private boolean isSynced;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public WristSubsystem() {
    super(WristState.HOME_WRIST);
    wristMotor = new TalonFX(Ports.WristPorts.WRIST_MOTOR);
    encoder = new DutyCycle(new DigitalInput(Ports.WristPorts.ENCODER));
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor_config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = WristConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = WristConstants.MotionMagicJerk;
    wristMotor.getConfigurator().apply(motor_config);
    tolerance = 0.04;
    brakeModeEnabled = false;
    isSynced = false;
  }
  protected WristState getNextState(WristState currentState) {
    if (getState() == WristState.HOME_WRIST && this.atGoal()) {
      wristMotor.setPosition(0.38);
      return WristState.INVERTED_IDLE;
    } else {
      return currentState;
    }
  }

  public void setL1Row() {
    if (RobotMode.getInstance().inHighL1Mode()) {
      WristPositions.L1 = WristPositions.L1_ROW2;
    } else {
      WristPositions.L1 = WristPositions.L1_ROW1;
    }
  }

   public boolean atGoal() {
    return true;
    // return switch (getState()) {
    //   case IDLE -> 
    //     MathUtil.isNear(WristPositions.IDLE, wristPosition, tolerance);
    //   case INVERTED_IDLE ->
    //     MathUtil.isNear(WristPositions.INVERTED_IDLE, wristPosition, tolerance);
    //   case PRE_L4 ->
    //     MathUtil.isNear(WristPositions.PRE_L4, wristPosition, tolerance);
    //   case L1 ->
    //     MathUtil.isNear(WristPositions.L1, wristPosition, tolerance);
    //   case L2 ->
    //     MathUtil.isNear(WristPositions.L2, wristPosition, tolerance);
    //   case L3 ->
    //     MathUtil.isNear(WristPositions.L3, wristPosition, tolerance);
    //   case CAPPED_L4 ->
    //     MathUtil.isNear(WristPositions.CAPPED_L4, wristPosition, tolerance);
    //   case L4_TRANSITION ->
    //     MathUtil.isNear(WristPositions.L4_TRANSITION, wristPosition, tolerance);
    //   case CORAL_STATION ->
    //     MathUtil.isNear(WristPositions.CORAL_STATION, wristPosition, tolerance);
    //   case HOME_WRIST ->
    //     motorCurrent > WristConstants.homingStallCurrent;
    //   case L4_WRIST ->
    //     MathUtil.isNear(WristPositions.L4_WRIST, wristPosition, tolerance);
    //   case INVERTED_CORAL_STATION ->
    //     MathUtil.isNear(WristPositions.INVERTED_CORAL_STATION, wristPosition, tolerance);
    //   case PRE_ALGAE_SCORE ->
    //     MathUtil.isNear(WristPositions.PRE_ALGAE_SCORE, wristPosition, tolerance);
    //   case SCORE_ALGAE ->
    //     MathUtil.isNear(WristPositions.ALGAE_SCORE, wristPosition, tolerance);
    //   case INTAKE_ALGAE ->
    //     MathUtil.isNear(WristPositions.ALGAE_INTAKE, wristPosition, tolerance);
    //   case ALGAE_FLICK ->
    //     MathUtil.isNear(WristPositions.ALGAE_FLICK, wristPosition, tolerance);
    //   case CAGE_FLIP ->
    //     MathUtil.isNear(WristPositions.CAGE_FLIP, wristPosition, tolerance);
    //   case PROCESSOR ->
    //     MathUtil.isNear(WristPositions.PROCESSOR, wristPosition, tolerance);
    //   case DISABLED->
    //     true;
    // };
  }

  public void setState(WristState newState) {
    setStateFromRequest(newState);
  }

  public boolean isIdle() {
    return getState() == WristState.INVERTED_IDLE;
  }

  // public void syncEncoder(){
  //   wristMotor.setPosition(absolutePosition);
  // }


    @Override
  public void collectInputs(){
    absolutePosition = encoder.getOutput();
    wristPosition = wristMotor.getPosition().getValueAsDouble();
    motorCurrent = wristMotor.getStatorCurrent().getValueAsDouble();
    DogLog.log(getName() + "/Wrist Position", wristPosition);
    DogLog.log(getName() + "/Wrist AtGoal", atGoal());
    DogLog.log(getName() + "/wrist current", motorCurrent);
    DogLog.log(getName() + "/encoder position", absolutePosition);
  }

  @Override
  public void periodic(){
    super.periodic();

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
      if (RobotManager.getInstance().getState() == RobotState.INVERTED_IDLE && RobotManager.getInstance().timeout(1) && !isSynced) {
        // syncEncoder();
        isSynced = true;
      }
      else if (RobotManager.getInstance().getState() != RobotState.INVERTED_IDLE) {
        isSynced = false;
      }
  }

  public void setWristPosition(double position){
    // wristMotor.setControl(motor_request.withPosition(position));
    wristMotor.setControl(motor_request.withPosition(0.376));
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
        case PRE_L4 -> {
          setWristPosition(WristPositions.PRE_L4);
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
        case L4_TRANSITION -> {
          setWristPosition(WristPositions.L4_TRANSITION);
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
        case L4_WRIST -> {
          setWristPosition(WristPositions.L4_WRIST);
        }
        case PRE_ALGAE_SCORE -> {
          setWristPosition(WristPositions.PRE_ALGAE_SCORE);
        }
        case INTAKE_ALGAE -> {
          setWristPosition(WristPositions.ALGAE_INTAKE);
        }
        case SCORE_ALGAE -> {
          setWristPosition(WristPositions.ALGAE_SCORE);
        }
        case ALGAE_FLICK -> {
          setWristPosition(WristPositions.ALGAE_FLICK);
        }
        case CAGE_FLIP -> {
          setWristPosition(WristPositions.CAGE_FLIP);
        }
        case PROCESSOR -> {
          setWristPosition(WristPositions.PROCESSOR);
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
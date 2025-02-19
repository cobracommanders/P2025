package frc.robot.commands;
import java.lang.Thread.State;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.FlagManager;
import frc.robot.StateMachine;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.kicker.KickerState;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorState;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.vision.LimelightLocalization;

public class RobotManager extends StateMachine<RobotState> {
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final ManipulatorSubsystem manipulator;
  public final WristSubsystem wrist;
  public final ElbowSubsystem elbow;
  public final DrivetrainSubsystem drivetrain;
  public final KickerSubsystem kicker;

  public boolean isHeightCapped = true;
  public boolean isInverted = false;
  public Timer timer = new Timer();

  public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

  public RobotManager() {
    super(RobotState.INVERTED_IDLE);
    this.elevator = ElevatorSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.manipulator = ManipulatorSubsystem.getInstance();
    this.wrist = WristSubsystem.getInstance();
    this.elbow = ElbowSubsystem.getInstance();
    this.kicker = KickerSubsystem.getInstance();
    this.drivetrain = DrivetrainSubsystem.getInstance();
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    flags.log();
    RobotState nextState = currentState;
    if (DriverStation.isDisabled() && DriverStation.isAutonomous()){
        nextState = RobotState.INVERTED_IDLE;
    }
    if(DriverStation.isDisabled() && DriverStation.isTeleop()){
      nextState = RobotState.PREPARE_HOMING;
    }
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case APPLY_HEIGHT_CAP:
          isHeightCapped = true;
          break;
        case REMOVE_HEIGHT_CAP:
          isHeightCapped = false;
          break;
        case IDLE:
          if (!currentState.ignoreRequests) {
            nextState = (!currentState.inverted) ? RobotState.PREPARE_IDLE : RobotState.PREPARE_IDLE_FROM_INVERTED;
          }
          break;
        case INVERTED_IDLE:
          if (!currentState.ignoreRequests) {
            nextState = (currentState.inverted) ? RobotState.PREPARE_INVERTED_IDLE : RobotState.PREPARE_INVERTED_FROM_IDLE;
          }
          break;
        case CORAL_STATION:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_CORAL_STATION;
          }
          break;
        case INVERTED_CORAL_STATION:
          if (!currentState.ignoreRequests && currentState.inverted) {
            nextState = RobotState.PREPARE_INVERTED_CORAL_STATION;
          }
          break;
        case L1:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PREPARE_L1;
          }
          break;
        case L2:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L2;
          }
          break;
        case L3:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L3;
          }
          break;
        case L4:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = isHeightCapped ? RobotState.CAPPED_L4 : RobotState.PREPARE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (currentState == RobotState.IDLE) {
            nextState = RobotState.DEEP_CLIMB;
          }
          break;
        case HOMING:
          nextState = RobotState.HOMING_STAGE_1_ELEVATOR;
          break;
        case SCORE:
          switch (nextState) {
            case WAIT_L1:
              nextState = RobotState.SCORE_L1;
              break;
            case WAIT_L2:
              nextState = RobotState.SCORE_L2;
              break;
            case WAIT_L3:
              nextState = RobotState.SCORE_L3;
              break;
            case WAIT_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case CAPPED_L4:
              nextState = RobotState.SCORE_L4;
            case DEEP_CLIMB:
            default:
              break;
          }
      }
    }
    //automatic transitions
    // DogLog.log(getName() + "/State After Flags", currentState);
    switch (currentState) {
      case WAIT_L2:
      case IDLE:
      case WAIT_INVERTED_IDLE:
      case WAIT_DEEP_CLIMB:
      case WAIT_IDLE:
      case WAIT_L1:
      case DEEP_CLIMB:
      case INTAKE_CORAL_STATION:
      case INVERTED_IDLE:
      case REMOVE_ALGAE:
      case WAIT_L3:
        break;
      
      case INVERTED_INTAKE_CORAL_STATION:
        if (timeout(5)) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
      case PREPARE_L1:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L1;
        }
        break;
      case PREPARE_L2:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.WAIT_L2;
        }
        break;
      case PREPARE_L3:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.WAIT_L3;
      }
        break;
      case CAPPED_L4:
        if (!isHeightCapped) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L4:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.L4_ELBOW;
        }
        break;
      case WAIT_L4:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        }
        break;
        case L4_ELBOW:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.WAIT_L4;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.INVERTED_INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.IDLE;
        }
        break;
      case PREPARE_INVERTED_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L1:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case SCORE_L2:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L3:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L4:
        if (timeout(1)) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case PREPARE_INVERTED_FROM_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_IDLE_FROM_INVERTED:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal()) {
          nextState = RobotState.PREPARE_IDLE;
        }
      case PREPARE_IDLE_FROM_SCORE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal() && currentState == RobotState.SCORE_L4) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_DEEP_CLIMB:
        if (climber.atGoal()) {
          nextState = RobotState.WAIT_DEEP_CLIMB;
        }
        break;
      case PREPARE_HOMING:
          if (DriverStation.isEnabled()) {
            nextState = RobotState.HOMING_STAGE_1_ELEVATOR;
          }
        break;
      case HOMING_STAGE_1_ELEVATOR:
        if (elevator.isIdle()) {
          nextState = RobotState.HOMING_STAGE_2_ELBOW;
        }
        break;
      case HOMING_STAGE_2_ELBOW:
        if (elbow.isIdle()) {//change to isIdle
          nextState = RobotState.HOMING_STAGE_3_WRIST;
        }
        break;
      case HOMING_STAGE_3_WRIST:
        if (wrist.isIdle()) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
    }
    DogLog.log(getName() + "/AtGoal", elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && manipulator.atGoal());
    flags.clear();
    return nextState;
  };
  

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
          case IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
            kicker.setState(KickerState.IDLE);
          }
          case SCORE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
            kicker.setState(KickerState.REMOVE_ALGAE);   
          }
          case SCORE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
            kicker.setState(KickerState.REMOVE_ALGAE);
          }
          case SCORE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L4);
            wrist.setState(WristState.L4);
            elbow.setState(ElbowState.L4);
            kicker.setState(KickerState.REMOVE_ALGAE);
          }
          case SCORE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L4);
            wrist.setState(WristState.L4);
            elbow.setState(ElbowState.L4);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
            kicker.setState(KickerState.IDLE);
          }
          case INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_INVERTED_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
            kicker.setState(KickerState.IDLE);
          }
          case INVERTED_INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
            kicker.setState(KickerState.IDLE);
          }
          case PREPARE_DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.PREPARE_DEEP_CLIMB);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }
          case DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }

          case CAPPED_L4 -> {
            elevator.setState(ElevatorState.CAPPED_L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAPPED_L4);
            elbow.setState(ElbowState.CAPPED_L4);
            kicker.setState(KickerState.REMOVE_ALGAE);
          }

          case L4_ELBOW -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.L4);
            elbow.setState(ElbowState.L4_ELBOW);
            kicker.setState(KickerState.REMOVE_ALGAE);
          }

          // case CAPPED_L3 -> {
          //   elevator.setState(ElevatorState.CAPPED_L3);
          //   climber.setState(ClimberState.IDLE);
          //   manipulator.setState(ManipulatorState.IDLE);
          //   wrist.setState(WristState.CAPPED_L3);
          //   elbow.setState(ElbowState.CAPPED_L3);
          //   kicker.setState(KickerState.REMOVE_ALGAE);
          // }

          case PREPARE_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }

          case PREPARE_IDLE_FROM_SCORE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.AFTER_L4);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }

          case PREPARE_IDLE_FROM_INVERTED -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.AFTER_INTAKE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
            kicker.setState(KickerState.IDLE);
          }

          case PREPARE_INVERTED_FROM_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.AFTER_INTAKE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            kicker.setState(KickerState.IDLE);
          }

          case PREPARE_INVERTED_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
            kicker.setState(KickerState.IDLE);
          }
          case HOMING_STAGE_1_ELEVATOR -> {
            elevator.setState(ElevatorState.HOME_ELEVATOR);
            wrist.setState(WristState.DISABLED);
            elbow.setState(ElbowState.DISABLED);
            manipulator.setState(ManipulatorState.IDLE);
            kicker.setState(KickerState.IDLE);
          }
          case HOMING_STAGE_2_ELBOW -> {
            elbow.setState(ElbowState.HOME_ELBOW);
            wrist.setState(WristState.DISABLED);
            manipulator.setState(ManipulatorState.IDLE);
            kicker.setState(KickerState.IDLE);

          }
          case HOMING_STAGE_3_WRIST -> {
            wrist.setState(WristState.HOME_WRIST);
            manipulator.setState(ManipulatorState.IDLE);
            kicker.setState(KickerState.IDLE);

          }
          case WAIT_L2 -> {
            kicker.setState(KickerState.REMOVE_ALGAE);
          }
          case WAIT_L3 -> {
            kicker.setState(KickerState.REMOVE_ALGAE);
          }
          case WAIT_L4 -> {
            kicker.setState(KickerState.REMOVE_ALGAE);
            elbow.setState(ElbowState.L4_ELBOW);
          }
          case INVERTED_IDLE,
            WAIT_DEEP_CLIMB, 
            WAIT_IDLE, 
            WAIT_INVERTED_IDLE, 
            WAIT_L1,
            PREPARE_HOMING,
            REMOVE_ALGAE -> {}
          }
      }

  @Override
  public void periodic() {
    super.periodic(); 
    DogLog.log(getName() + "/Is capped", isHeightCapped);
  }

  public void prepareIdleRequest() {
    flags.check(RobotFlag.IDLE);
  }

  public void prepareInvertedIdleRequest(){
    flags.check(RobotFlag.INVERTED_IDLE);
  }

  public void prepareL1Request() {
    flags.check(RobotFlag.L1);
  }

  public void prepareL2Request() {
    flags.check(RobotFlag.L2);
  }

  public void prepareL3Request() {
    flags.check(RobotFlag.L3);
  }
  
  public void prepareL4Request() {
    flags.check(RobotFlag.L4);
  }
  
  public void prepareDeepClimbRequest() {
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void prepareCoralStationRequest() {
    flags.check(RobotFlag.CORAL_STATION);
  }

  public void prepareInvertedCoralStationRequest() {
    flags.check(RobotFlag.INVERTED_CORAL_STATION);
  }

  public void scoreRequest() {
    flags.check(RobotFlag.SCORE);
  }

  public void climbRequest(){
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void applyHeightCapRequest(){
    flags.check(RobotFlag.APPLY_HEIGHT_CAP);
  }

  public void removeHeightCapRequest(){
    flags.check(RobotFlag.REMOVE_HEIGHT_CAP);
  }

  public void homeRequest(){
    flags.check(RobotFlag.HOMING);
  }

  public void autoReefAlignRequest(){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_REEF_ALIGN);
  }

  public void autoCoralStationAlignRequest(){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_CORAL_STATION_ALIGN);
  }

  public void stopScoringRequest() {
    switch (getState()) {
      case DEEP_CLIMB-> {}
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
  private static RobotManager instance;
  
  public static RobotManager getInstance() {
    if (instance == null) instance = new RobotManager(); // Make sure there is an instance (this will only run once)
    return instance;
}

}
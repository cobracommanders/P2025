package frc.robot.commands;
import static edu.wpi.first.wpilibj2.command.Commands.none;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.Controls;
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
import frc.robot.subsystems.manipulator.ManipulatorState;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final ManipulatorSubsystem manipulator;
  public final WristSubsystem wrist;
  public final ElbowSubsystem elbow;
  public final DrivetrainSubsystem drivetrain;

  public boolean isHeightCapped = true;
  public GameMode currentGameMode = GameMode.CORAL;
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
    this.drivetrain = DrivetrainSubsystem.getInstance();
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    flags.log();
    RobotState nextState = currentState;
    // if (DriverStation.isDisabled() && DriverStation.isAutonomous()){
    //     nextState = RobotState.INVERTED_IDLE;
    // }
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case ALGAE_MODE:
          currentGameMode = GameMode.ALGAE;
          break;
        case CORAL_MODE:
          currentGameMode = GameMode.CORAL;
          break;
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
            if (RobotMode.getInstance().inCoralMode()) {
              if (currentState == RobotState.INVERTED_INTAKE_CORAL_STATION) {
                nextState = RobotState.POST_INVERTED_CORAL_STATION_INTAKE;
              } else {
                nextState = (currentState.inverted) ? RobotState.PREPARE_INVERTED_IDLE : RobotState.PREPARE_INVERTED_FROM_IDLE;
              }
            }
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
            nextState = RobotState.PRE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PREPARE_DEEP_CLIMB;
          }
          break;
        case PROCESSOR:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PREPARE_PROCESSOR;
          }
          break;
        case CLIMB_UNWIND:
          nextState = RobotState.DEEP_CLIMB_UNWIND;
          break;
        case CLIMB_IDLE:
          nextState = RobotState.DEEP_CLIMB_WAIT;
          break;
        case CLIMB_RETRACT:
          nextState = RobotState.DEEP_CLIMB_RETRACT;
          break;
        case INTAKE_ALGAE:
          if (currentState == RobotState.WAIT_REMOVE_ALGAE_HIGH){
            nextState = RobotState.REMOVE_ALGAE_HIGH;
          }
          else if (currentState == RobotState.WAIT_REMOVE_ALGAE_LOW){
            nextState = RobotState.REMOVE_ALGAE_LOW;
          }
          break;

        case STOP_INTAKE_ALGAE:
          if (currentState == RobotState.REMOVE_ALGAE_HIGH){
            nextState = RobotState.WAIT_REMOVE_ALGAE_HIGH;
          }
          else if (currentState == RobotState.REMOVE_ALGAE_LOW){
            nextState = RobotState.WAIT_REMOVE_ALGAE_LOW;
          }
          else if(currentState == RobotState.SCORE_ALGAE){
            nextState = RobotState.WAIT_REMOVE_ALGAE_HIGH; 
          }
          break;
        case ALGAE_HIGH:
        if (!currentState.ignoreRequests && !currentState.inverted) {
          nextState = RobotState.PREPARE_REMOVE_ALGAE_HIGH;
        }
        break;
        case ALGAE_LOW:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_REMOVE_ALGAE_LOW;
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
            case L4_ELBOW:
              nextState = RobotState.SCORE_L4;
              break;
            case WAIT_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case CAPPED_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case SCORE_ALGAE_WAIT:
              nextState = RobotState.SCORE_ALGAE;
              break;
            case WAIT_PROCESSOR:
              nextState = RobotState.SCORE_PROCESSOR;
              break;
            default:
              break;
          }
      }
    }
    switch (currentState) {
      case WAIT_L2:
      case IDLE:
      case WAIT_IDLE:
      case WAIT_L1:
      case INTAKE_CORAL_STATION:
      case INVERTED_IDLE:
      case DEEP_CLIMB_WAIT:
      case WAIT_L3:
      case PRE_HEIGHT_L4:
      case WAIT_PROCESSOR:
        break;

      case DEEP_CLIMB_DEPLOY:
        if(timeout(0.1)){
          nextState = RobotState.DEEP_CLIMB_WAIT;
        } 
        break;
      case PRE_L4:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L1:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L1;
        }
        break;
      case PREPARE_L2:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L2;
        }
        break;
      case PREPARE_L3:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
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
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
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
        } else if (elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L4;
        }
        break;
      case PREPARE_REMOVE_ALGAE_HIGH:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        }
        break;
      case PREPARE_REMOVE_ALGAE_LOW:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_REMOVE_ALGAE_LOW;
        }
        break;
      case PREPARE_PROCESSOR:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_PROCESSOR;
        }
        break;
      case PREPARE_SCORE_ALGAE:
        if(isHeightCapped) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.SCORE_ALGAE_WAIT;
        }
        break;
      case SCORE_ALGAE_WAIT:
        if(isHeightCapped) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        }
        break;
      case WAIT_REMOVE_ALGAE_LOW:
        if(!isHeightCapped){
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case WAIT_REMOVE_ALGAE_HIGH:
        if(!isHeightCapped){
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INVERTED_INTAKE_CORAL_STATION;
        }
        break;
      case INVERTED_INTAKE_CORAL_STATION:
        if (ManipulatorSubsystem.getInstance().hasCoral() && timeout(0.6) && DriverStation.isAutonomous()) {
          nextState = RobotState.POST_INVERTED_CORAL_STATION_INTAKE;
        }
        break;
      case POST_INVERTED_CORAL_STATION_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_DEEP_CLIMB:
        if (elbow.atGoal() && elevator.atGoal() && wrist.atGoal()) {
          nextState = RobotState.DEEP_CLIMB_DEPLOY;
        }
        break;
      case PREPARE_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.IDLE;
        }
        break;
      case PREPARE_INVERTED_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
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
        if ((timeout(2) && DriverStation.isTeleop()) || (timeout(0.35) && DriverStation.isAutonomous())) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case REMOVE_ALGAE_HIGH:
        if(!isHeightCapped) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case REMOVE_ALGAE_LOW:
        if(!isHeightCapped) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case SCORE_ALGAE:
        if ((timeout(3) && DriverStation.isTeleop() || (timeout(1.5) && DriverStation.isAutonomous()))) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case SCORE_PROCESSOR:
        if ((timeout(3) && DriverStation.isTeleop() || (timeout(1) && DriverStation.isAutonomous()))) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case PREPARE_INVERTED_FROM_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_IDLE_FROM_INVERTED:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_IDLE;
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
      case DEEP_CLIMB_RETRACT:
        if (timeout(5)){
          nextState = RobotState.DEEP_CLIMB_WAIT;
        }
        break;
      case DEEP_CLIMB_UNWIND:
        if (timeout(5)){
          nextState = RobotState.DEEP_CLIMB_WAIT;
        }

        break;
    }
    DogLog.log(getName() + "/AtGoal", elevator.atGoal() && elbow.atGoal() && wrist.atGoal());
    DogLog.log(getName() + "/isCoralMode", Controls.getInstance().isCoralMode);
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
          }
          case PRE_L4 -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }
          case PREPARE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case SCORE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case PREPARE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
          }
          case SCORE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
          }
          case PREPARE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case SCORE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case PREPARE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L4);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.L4);
          }
          case SCORE_L4 -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L4);
            wrist.setState(WristState.L4_WRIST);
            elbow.setState(ElbowState.L4);
          }
          case PREPARE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case POST_INVERTED_CORAL_STATION_INTAKE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case PREPARE_INVERTED_CORAL_STATION -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case INVERTED_INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case PREPARE_DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }

          case DEEP_CLIMB_DEPLOY -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_DEPLOY);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
          }

          case DEEP_CLIMB_WAIT -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_WAIT);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
          }

          case DEEP_CLIMB_RETRACT -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_RETRACT);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
          }

          case DEEP_CLIMB_UNWIND -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_UNWIND);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
          }

          case CAPPED_L4 -> {
            elevator.setState(ElevatorState.CAPPED_L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.CAPPED_L4);
          }

          case L4_ELBOW -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.AFTER_INTAKE);
            wrist.setState(WristState.L4_WRIST);
            elbow.setState(ElbowState.L4_ELBOW);
          }

          case PREPARE_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }

          case PREPARE_IDLE_FROM_INVERTED -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
          }

          case PREPARE_INVERTED_FROM_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }

          case PREPARE_INVERTED_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
          }
          case PREPARE_REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.HIGH_ALGAE);
          }
          case PREPARE_REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
          }
          case PREPARE_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }

          case WAIT_REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.HIGH_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
          }
          case WAIT_REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.LOW_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
          }
          case WAIT_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.PROCESSOR);
            wrist.setState(WristState.PROCESSOR);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
          }

          case REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.HIGH_ALGAE);
          }
          case REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
          }
          case SCORE_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.PROCESSOR);
            wrist.setState(WristState.PROCESSOR);
            manipulator.setState(ManipulatorState.SCORE_PROCESSOR);
          }

          case PREPARE_SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PRE_ALGAE_SCORE);
            elbow.setState(ElbowState.PRE_SCORE_ALGAE);
          }

          case SCORE_ALGAE_WAIT -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.SCORE_ALGAE);
            elbow.setState(ElbowState.L4);
          }
          

          case SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.SCORE_ALGAE);
            wrist.setState(WristState.ALGAE_FLICK);
            elbow.setState(ElbowState.L4);
          }
          
          case HOMING_STAGE_1_ELEVATOR -> {
            elevator.setState(ElevatorState.HOME_ELEVATOR);
            wrist.setState(WristState.DISABLED);
            elbow.setState(ElbowState.DISABLED);
          }
          case HOMING_STAGE_2_ELBOW -> {
            elbow.setState(ElbowState.HOME_ELBOW);
            wrist.setState(WristState.DISABLED);
          }
          case HOMING_STAGE_3_WRIST -> {
            wrist.setState(WristState.HOME_WRIST);
          }
          case WAIT_L2 -> {
            manipulator.setState(ManipulatorState.PRE_SCORE);
          }
          case WAIT_L3 -> {
            manipulator.setState(ManipulatorState.PRE_SCORE);
          }
          case WAIT_L4 -> {
            manipulator.setState(ManipulatorState.IDLE);
            elbow.setState(ElbowState.L4_ELBOW);
            wrist.setState(WristState.L4_WRIST);
            elevator.setState(ElevatorState.L4_MAX);
          }

          case INVERTED_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            //wrist.syncEncoder();
          }
          case 
            WAIT_IDLE, 
            WAIT_L1,
            PRE_HEIGHT_L4,
            PREPARE_HOMING -> {}
          }
      }

  @Override
  public void periodic() {
    super.periodic(); 
    DogLog.log(getName() + "/is Coral Mode", RobotMode.getInstance().inCoralMode());
    DogLog.log(getName() + "/Is capped", isHeightCapped);
    //DogLog.log(getName() + "Active Command", elevator.getCurrentCommand().toString());
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

  public void prepareAlgaeHighRequest() {
    flags.check(RobotFlag.ALGAE_HIGH);
  }

  public void prepareAlgaeLowRequest() {
    flags.check(RobotFlag.ALGAE_LOW);
  }
  
  public void prepareDeepClimbRequest() {
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void setProcessorRequest() {
    flags.check(RobotFlag.PROCESSOR);
  }

  public void prepareCoralStationRequest() {
    flags.check(RobotFlag.CORAL_STATION);
  }

  public void prepareInvertedCoralStationRequest() {
    flags.check(RobotFlag.INVERTED_CORAL_STATION);
  }

  public void intakeAlgaeRequest(){
    flags.check(RobotFlag.INTAKE_ALGAE);
  }

  public void stopIntakeAlgaeRequest(){
    flags.check(RobotFlag.STOP_INTAKE_ALGAE);
  }

  public void scoreRequest() {
    flags.check(RobotFlag.SCORE);
  }

  public void algaeScoreRequest() {
    flags.check(RobotFlag.SCORE);
  }

  public void climbRequest(){
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void climbUnwindRequest(){
    flags.check(RobotFlag.CLIMB_UNWIND);
  }

  public void climbIdleRequest(){
    flags.check(RobotFlag.CLIMB_IDLE);
  }

  public void climbRetractRequest(){
    flags.check(RobotFlag.CLIMB_RETRACT);
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

  public void algaeModeRequest(){
    flags.check(RobotFlag.ALGAE_MODE);
  }

  public void coralModeRequest(){
    flags.check(RobotFlag.CORAL_MODE);
  }

  public void autoReefAlignRequest(){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_REEF_ALIGN_1);
  }

  public void autoAlgaeAlignRequest(){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_ALGAE_ALIGN);
  }

  public void autoCoralStationAlignRequest(){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_CORAL_STATION_ALIGN_1);
  }

  public void stopScoringRequest() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
  private static RobotManager instance;
  
  public static RobotManager getInstance() {
    if (instance == null) instance = new RobotManager(); // Make sure there is an instance (this will only run once)
    return instance;
}

}
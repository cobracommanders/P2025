package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.vision.LimelightLocalization;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.List;

import dev.doglog.DogLog;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;
  

  public RobotCommands() {
    this.robot = RobotManager.getInstance();
    var requirementsList = List.of(robot.elevator, robot.climber, robot.wrist, robot.elbow, robot.manipulator);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }
  
  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.PREPARE_INVERTED_FROM_IDLE));
  }

  public Command algaeScoreCommand() {
    return Commands.runOnce(robot::algaeScoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command L4ScoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.PREPARE_INVERTED_FROM_IDLE));
  }

  public Command retractClimbCommand(){
    return Commands.runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.DEEP_CLIMB_RETRACT), requirements)
    .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
  }
  public Command L2MultiCommand() {
    return new ConditionalCommand(L2Command(), algaeIdleCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  public Command L1MultiCommand() {
    return new ConditionalCommand(L1Command(), algaeIdleCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  public Command L1Command() {
    return new ConditionalCommand(invertIdleCommand().andThen(Commands.runOnce(robot::prepareL1Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L1)), Commands.runOnce(robot::prepareL1Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L1)), () -> !robot.getState().inverted);
    // if (!robot.getState().inverted) {
      // return invertIdleCommand()
      //   .andThen(Commands.runOnce(robot::prepareL1Request, requirements))
      //   .andThen(robot.waitForState(RobotState.WAIT_L1));
    // }else {
      // return Commands.runOnce(robot::prepareL1Request, requirements)
      //   .andThen(robot.waitForState(RobotState.WAIT_L1));
    // }
  }

  public Command L2Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL2Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L2)), Commands.runOnce(robot::prepareL2Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L2)), () -> robot.getState().inverted);
    //if (robot.getState().inverted) {
    //   return alternateIdleCommand()
    //     .andThen(Commands.runOnce(robot::prepareL2Request, requirements))
    //     .andThen(robot.waitForState(RobotState.WAIT_L2));
    // } else {
    //   return Commands.runOnce(robot::prepareL2Request, requirements)
    //       .andThen(robot.waitForState(RobotState.WAIT_L2));
    // }
  }

  public Command L3Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL3Request, requirements)).andThen(robot.waitForState(RobotState.WAIT_L3)), Commands.runOnce(robot::prepareL3Request, requirements).andThen(robot.waitForState(RobotState.WAIT_L3)), () -> robot.getState().inverted);
    // if (robot.getState().inverted) {
    //   return alternateIdleCommand()
    //     .andThen(Commands.runOnce(robot::prepareL3Request, requirements))
    //     .andThen(robot.waitForState(RobotState.WAIT_L3));
    // } else {
    //   return Commands.runOnce(robot::prepareL3Request, requirements)
    //     .andThen(robot.waitForState(RobotState.WAIT_L3));
    // }
  }
  public Command L4Command() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareL4Request, requirements)), Commands.runOnce(robot::prepareL4Request, requirements), () -> robot.getState().inverted);
    // if (robot.getState().inverted) {
    //   return alternateIdleCommand()
    //   .andThen(Commands.runOnce(robot::prepareL4Request, requirements));
    //   // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
    // } else {
    //   return Commands.runOnce(robot::prepareL4Request, requirements);
    //   // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
    // }
  }
  public Command lowAlgaeCommand() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)), Commands.runOnce(robot::prepareAlgaeLowRequest, requirements), () -> robot.getState().inverted);
    // if (robot.getState().inverted) {
    //   return algaeIdleCommand() // go to non-inverted idle
    //   .andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)); // Prepare CS (non-inverted)
    //   // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
    // } else {
    //   return Commands.runOnce(robot::prepareAlgaeLowRequest, requirements);
    //   // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
    // }
  }
  public Command highAlgaeCommand() {
    return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::prepareAlgaeHighRequest, requirements)), Commands.runOnce(robot::prepareAlgaeHighRequest, requirements), () -> robot.getState().inverted);
  }

  public Command setProcessorCommand() {
    return Commands.runOnce(robot::setProcessorRequest, requirements);
    //return new ConditionalCommand(algaeIdleCommand().andThen(Commands.runOnce(robot::setProcessorRequest, requirements)), Commands.runOnce(robot::setProcessorRequest, requirements), () -> robot.getState().inverted);
  }

  public Command ProcessorCommand() {
    return new ConditionalCommand(L1Command(), setProcessorCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command LowReefCommand() {
    DogLog.log("coral mode", RobotManager.getInstance().currentGameMode == GameMode.CORAL);
    return new ConditionalCommand(L3Command(), lowAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command HighReefCommand() {
    return new ConditionalCommand(L4Command(), highAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  // public Command algaeHighCommand() {
  //   if (robot.getState().inverted) {
  //     return idleCommand() // go to non-inverted idle
  //     .andThen(Commands.runOnce(robot::prepareAlgaeHighRequest, requirements)); // Prepare CS (non-inverted)
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
  //   } else {
  //     return Commands.runOnce(robot::prepareAlgaeHighRequest, requirements);
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
  //   }
  // }

  // public Command algaeLowCommand() {
  //   if (robot.getState().inverted) {
  //     return idleCommand() // go to non-inverted idle
  //     .andThen(Commands.runOnce(robot::prepareAlgaeLowRequest, requirements)); // Prepare CS (non-inverted)
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4)); // Goes back to idle when we're done intaking
  //   } else {
  //     return Commands.runOnce(robot::prepareAlgaeLowRequest, requirements);
  //     // .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.WAIT_L4));
  //   }
  // }

  public Command algaeIdleCommand() {
    return Commands.runOnce(robot::prepareIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command invertIdleCommand() {
      return Commands.runOnce(robot::prepareInvertedIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
  }

  public Command idleCommand() {
    return new ConditionalCommand(invertIdleCommand(), stopIntakeAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }

  public Command climbCommand() {
    // if (robot.getState().inverted) {
      return algaeIdleCommand()
      .andThen(Commands.runOnce(robot::climbRequest, requirements))
      .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));
    // } else{
    // return Commands.runOnce(robot::climbRequest, requirements)
    //     .andThen(robot.waitForState(RobotState.DEEP_CLIMB_WAIT));    
    // }
  }

  public Command climbUnwindCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbUnwindRequest), climbCommand(), () -> RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT);
  }

  public Command climbIdleCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbIdleRequest), climbCommand(), () -> RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT || RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_RETRACT || RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_UNWIND);
  }

  public Command climbRetractCommand() {
    return new ConditionalCommand(Commands.runOnce(robot::climbRetractRequest), climbCommand(), () -> RobotManager.getInstance().getState() == RobotState.DEEP_CLIMB_WAIT);
  }

  public Command removeHeightCapCommand() {
    return Commands.runOnce(robot::removeHeightCapRequest);
  }

  public Command applyHeightCapCommand() {
    return Commands.runOnce(robot::applyHeightCapRequest);
  }

  public Command alternateIntakeCommand() {
    if (robot.getState().inverted) {
      return algaeIdleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareCoralStationRequest, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.IDLE)); // Goes back to idle when we're done intaking
    }else {
      return Commands.runOnce(robot::prepareCoralStationRequest, requirements)
          .andThen(robot.waitForState(RobotState.IDLE));
    }
  }

  public Command invertedIntakeCommand() {
      if (!robot.getState().inverted) {
        return invertIdleCommand() // go to non-inverted idle
          .andThen(Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)) // Prepare CS (inverted)
          .andThen(robot.waitForState(RobotState.INVERTED_IDLE)); // Goes back to inverted idle when we're done intaking
      } else {
        return Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)
            .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
      }
  }
  public Command intakeAlgaeCommand() {
    return runOnce(robot::intakeAlgaeRequest, requirements);
  }

  public Command stopIntakeAlgaeCommand() {
    return runOnce(robot::stopIntakeAlgaeRequest, requirements);
  }

  public Command intakeCommand() {
    return new ConditionalCommand(invertedIntakeCommand(), intakeAlgaeCommand(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
    // if (RobotManager.getInstance().currentGameMode == GameMode.CORAL) {
    //   return invertedIntakeCommand();
    // } else {
    //   return intakeAlgaeCommand();
    // }
  }



  public Command autoCoralStationAlign(){
    return Commands.runOnce(robot::autoCoralStationAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP));
  }

  public Command autoAlignCommand(){
    return new ConditionalCommand(autoReefAlign(), autoAlgaeAlign(), () -> RobotManager.getInstance().currentGameMode == GameMode.CORAL);
  }
  
  public Command autoReefAlign(){
    return new ConditionalCommand(Commands.runOnce(robot::autoReefAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil(()-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP))
    ,none()
    ,() ->  DriverStation.isAutonomous() || CommandSwerveDrivetrain.getInstance().isNear(LimelightLocalization.getInstance().getAdjustedBranchPose(), 0.5));
  }

  public Command autoAlgaeAlign(){
    return new ConditionalCommand(Commands.runOnce(robot::autoAlgaeAlignRequest, CommandSwerveDrivetrain.getInstance())
    .andThen(Commands.waitUntil((

    )-> DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP))
    ,none()
    ,() ->  DriverStation.isAutonomous() || CommandSwerveDrivetrain.getInstance().isNear(LimelightLocalization.getInstance().getAdjustedAlgaePose(), 0.5));
  }

  public Command setDrivetrainAuto(){
    return Commands.runOnce(()-> DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO));
  }

  public Command setDrivetrainTeleop(){
    return Commands.runOnce(()-> DrivetrainSubsystem.getInstance().setState(DrivetrainState.TELEOP));
  }

  public Command homeCommand(){
    return Commands.runOnce(robot::homeRequest, requirements)
      .andThen(robot.waitForState(RobotState.PREPARE_HOMING));    
  }

  public Command algaeModeCommand(){
    return algaeIdleCommand().andThen(Commands.runOnce(robot::algaeModeRequest));
  }

  public Command coralModeCommand(){ 
    return invertIdleCommand().andThen(Commands.runOnce(robot::coralModeRequest));
  }


  // public Command climberRetract(){
  //   if()
  // }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands() {
    this.robot = RobotManager.getInstance();
    var requirementsList = List.of(robot.elevator, robot.climber, robot.wrist, robot.elbow, robot.manipulator, robot.kicker);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
  }

  public Command L1Command() {
    if (!robot.getState().inverted) {
      return invertIdleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL1Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L1)); // Goes back to idle when we're done intaking
    }else {
      return Commands.runOnce(robot::prepareL1Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L1));
    }
  }
  
  public Command L2Command() {
    if (robot.getState().inverted) {
      return idleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL2Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L2)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL2Request, requirements)
          .andThen(robot.waitForState(RobotState.WAIT_L2));
    }
  }

  public Command L3Command() {
    if (robot.getState().inverted) {
      return idleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL3Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.WAIT_L3)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL3Request, requirements)
          .andThen(robot.waitForState(RobotState.WAIT_L3));
    }
  }

  public Command L4Command() {
    if (robot.getState().inverted) {
      return idleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL4Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.PREPARE_L4)); // Goes back to idle when we're done intaking
    } else {
    return Commands.runOnce(robot::prepareL4Request, requirements)
      .andThen(robot.waitForState(RobotManager.getInstance().isHeightCapped == true ? RobotState.CAPPED_L4 : RobotState.PREPARE_L4));
    }
  }

  public Command L4AutoCommand() {
    if (robot.getState().inverted) {
      return idleCommand() // go to non-inverted idle
        .andThen(Commands.runOnce(robot::prepareL4Request, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.PREPARE_L4)); // Goes back to idle when we're done intaking
    } else {
      return Commands.runOnce(robot::prepareL2Request, requirements)
          .andThen(robot.waitForState(RobotState.PREPARE_L4));
    }
  }

  public Command idleCommand() {
    return Commands.runOnce(robot::prepareIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command invertIdleCommand() {
    return Commands.runOnce(robot::prepareInvertedIdleRequest, requirements)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
  }

  public Command climbCommand() {
    return Commands.runOnce(robot::climbRequest, requirements)
        .andThen(robot.waitForState(RobotState.DEEP_CLIMB));    
  }

  public Command removeHeightCapCommand() {
    return Commands.runOnce(robot::removeHeightCapRequest);
  }

  public Command applyHeightCapCommand() {
    return Commands.runOnce(robot::applyHeightCapRequest);
  }

  public Command intakeCommand() {
    if (robot.getState().inverted) {
      return idleCommand() // go to non-inverted idle
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

  public Command autoCoralStationAlign(){
    return Commands.runOnce(robot::autoCoralStationAlignRequest, requirements);
  }

  public Command autoReefAlign(){
    return Commands.runOnce(robot::autoReefAlignRequest, requirements);
  }

  public Command homeCommand(){
    return Commands.runOnce(robot::homeRequest, requirements)
      .andThen(robot.waitForState(RobotState.PREPARE_HOMING));    
  }
}

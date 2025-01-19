package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

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
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command L1Command() {
    return Commands.runOnce(robot::prepareL1Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L1));
  }
  
  public Command L2Command() {
    return Commands.runOnce(robot::prepareL2Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L2));
  }

  public Command L3Command() {
    return Commands.runOnce(robot::prepareL3Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L3));
  }

  public Command L4Command() {
    return Commands.runOnce(robot::prepareL4Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_L4));
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
    return Commands.runOnce(robot::removeHeightCapRequest, requirements);
  }

  public Command applyHeightCapCommand() {
    return Commands.runOnce(robot::applyHeightCapRequest, requirements);
  }

  // public Command intakeCommand() {
  //   Command intakeCommand = Commands.none();
  //   if (robot.getState().inverted) {
  //     intakeCommand = idleCommand();
  //   }
  //   intakeCommand = intakeCommand.andThen(Commands.runOnce(robot::prepareCoralStationRequest, requirements))
  //     .andThen(robot.waitForState(RobotState.IDLE));
  //   return intakeCommand;
  // }

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
        .andThen(Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)) // Prepare CS (non-inverted)
        .andThen(robot.waitForState(RobotState.INVERTED_IDLE)); // Goes back to idle when we're done intaking
    }else {
      return Commands.runOnce(robot::prepareInvertedCoralStationRequest, requirements)
          .andThen(robot.waitForState(RobotState.INVERTED_IDLE));
    }
  }
}

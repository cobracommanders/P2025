package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotState;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.vision.LimelightLocalization;
import frc.robot.vision.LimelightState;
import frc.robot.vision.LimelightSubsystem;

import java.lang.reflect.Field;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;


public class Robot extends TimedRobot{
    public static final double DEFAULT_PERIOD = 0.02;

    public static RobotManager robotManager = RobotManager.getInstance();
    public static RobotCommands robotCommands = new RobotCommands();

    public static Optional<Alliance> alliance = Optional.empty();
    public static final Controls controls = new Controls();

    private SendableChooser<Command> autoChooser;

    public Robot() {
    }

    @Override
    public void robotInit() {
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        WristSubsystem.getInstance().wristMotor.setPosition(0.38);
        LimelightSubsystem.getInstance().setState(LimelightState.DISABLED);
        // ElevatorSubsystem.getInstance().setTeleopConfig();

        NamedCommands.registerCommand("idle", Robot.robotCommands.algaeIdleCommand());
        NamedCommands.registerCommand("inverted idle", Robot.robotCommands.invertIdleCommand());
        NamedCommands.registerCommand("score", Robot.robotCommands.scoreCommand());
        NamedCommands.registerCommand("L1", Robot.robotCommands.L1Command());
        NamedCommands.registerCommand("L2", Robot.robotCommands.L2Command());
        NamedCommands.registerCommand("L3", Robot.robotCommands.L3Command());
        NamedCommands.registerCommand("L4", Robot.robotCommands.L4Command());
        NamedCommands.registerCommand("wait for inverted idle", robotManager.waitForState(RobotState.INVERTED_IDLE));
        NamedCommands.registerCommand("wait for idle", robotManager.waitForState(RobotState.IDLE));
        NamedCommands.registerCommand("wait for prepare inverted idle", robotManager.waitForState(RobotState.PREPARE_INVERTED_IDLE));
        NamedCommands.registerCommand("wait for post intake", robotManager.waitForState(RobotState.POST_INVERTED_CORAL_STATION_INTAKE));
        NamedCommands.registerCommand("wait for L4", robotManager.waitForState(RobotState.WAIT_L4));
        NamedCommands.registerCommand("wait for L4 elbow", robotManager.waitForState(RobotState.L4_ELBOW));
        NamedCommands.registerCommand("limelight state to auto reef", Commands.runOnce(() -> LimelightSubsystem.getInstance().setStateFromRequest(LimelightState.AUTO_REEF)));
        NamedCommands.registerCommand("limelight state to auto coral station", Commands.runOnce(() -> LimelightSubsystem.getInstance().setStateFromRequest(LimelightState.AUTO_CORAL_STATION)));
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());
        NamedCommands.registerCommand("apply height cap", Robot.robotCommands.applyHeightCapCommand());
        NamedCommands.registerCommand("auto coral station align", Robot.robotCommands.autoCoralStationAlign());
        NamedCommands.registerCommand("auto reef align", Robot.robotCommands.autoReefAlign());
        NamedCommands.registerCommand("auto algae align", Robot.robotCommands.autoAlgaeAlign());
        NamedCommands.registerCommand("climb", Robot.robotCommands.climbCommand());
        NamedCommands.registerCommand("intake", Robot.robotCommands.intakeCommand());
        NamedCommands.registerCommand("inverted intake", Robot.robotCommands.invertedIntakeCommand());
        NamedCommands.registerCommand("home", Robot.robotCommands.homeCommand());
        NamedCommands.registerCommand("set drivetrain auto", Robot.robotCommands.setDrivetrainAuto());
        NamedCommands.registerCommand("set coral mode", Robot.robotCommands.coralModeCommand());
        NamedCommands.registerCommand("set algae mode", Robot.robotCommands.algaeModeCommand());
        NamedCommands.registerCommand("enable supercycle", Robot.robotCommands.supercycleModeCommand());
        NamedCommands.registerCommand("enable regular cycle", Robot.robotCommands.regularCycleModeCommand());
        NamedCommands.registerCommand("low algae intake", Robot.robotCommands.lowAlgaeCommand());
        NamedCommands.registerCommand("high algae intake", Robot.robotCommands.highAlgaeCommand());
        NamedCommands.registerCommand("wait for low algae intake", robotManager.waitForState(RobotState.WAIT_REMOVE_ALGAE_LOW));
        NamedCommands.registerCommand("wait for high algae intake", robotManager.waitForState(RobotState.WAIT_REMOVE_ALGAE_HIGH));
        NamedCommands.registerCommand("wait for prepare algae score", robotManager.waitForState(RobotState.PREPARE_SCORE_ALGAE));
        NamedCommands.registerCommand("wait for algae score", robotManager.waitForState(RobotState.SCORE_ALGAE_WAIT));

        autoChooser = AutoBuilder.buildAutoChooser();
        LED led = new LED(robotManager);
        FieldConstants.getInstance().logBranches();
        FieldConstants.getInstance().logAlgae();
        FieldConstants.getInstance().logCoralStation();
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(autoChooser);
        if (alliance.isEmpty()) {
            alliance = DriverStation.getAlliance();
        }
        DogLog.log("Robot/Is autonomous", DriverStation.isAutonomous());
        DogLog.log("Robot/Is disabled", DriverStation.isDisabled());
        DogLog.log("Robot/Is teleop", DriverStation.isTeleop());
        DogLog.log("Robot/Is near high algae", FieldConstants.getInstance().isNearHighAlgae());
    }

    @Override
    public void disabledPeriodic() {
        LimelightSubsystem.getInstance().setState(LimelightState.DISABLED);
        alliance = DriverStation.getAlliance();
    }


    @Override
    public void teleopInit() {
        LimelightSubsystem.getInstance().setState(LimelightState.DRIVE);
        CommandScheduler.getInstance().schedule(Robot.robotCommands.setDrivetrainTeleop());
    }

    @Override
    public void teleopPeriodic() {
        
      }


    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void disabledInit() {
    }
    
    @Override
    public void autonomousInit() {
        
        if (autoChooser.getSelected() != null)
            autoChooser.getSelected().schedule();
            DogLog.log("Selected Auto", autoChooser.getSelected().getName());
    }
    @Override
    public void autonomousExit() {
        ElevatorSubsystem.getInstance().setTeleopConfig();
    }
    @Override
  public void autonomousPeriodic() {}


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
        }
    }
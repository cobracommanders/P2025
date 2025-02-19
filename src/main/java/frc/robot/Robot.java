package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RobotManager;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elbow.ElbowPositions;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.vision.LimelightLocalization;

import java.util.List;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;


public class Robot extends TimedRobot{
    public static final double DEFAULT_PERIOD = 0.02;

    public static RobotManager robotManager = RobotManager.getInstance();
    public static RobotCommands robotCommands = new RobotCommands();
    // public static int coordinateFlip = 1;
    // public static int rotationOffset = 0;

    public static Optional<Alliance> alliance = Optional.empty();
    public static final Controls controls = new Controls();

    // private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    // private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    // private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // private final ElbowSubsystem elbow = ElbowSubsystem.getInstance();
    // private final ManipulatorSubsystem manipulator = ManipulatorSubsystem.getInstance();
    // private final WristSubsystem wrist = WristSubsystem.getInstance();

    private SendableChooser<Command> autoChooser;

    public Robot() {
      //  DogLog.setOptions(new DogLogOptions().withCaptureNt(false).withNtPublish(true));
    }

    @Override
    public void robotInit() {
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        NamedCommands.registerCommand("idle", Robot.robotCommands.idleCommand());
        NamedCommands.registerCommand("inverted idle", Robot.robotCommands.invertIdleCommand());
        NamedCommands.registerCommand("score", Robot.robotCommands.scoreCommand());
        NamedCommands.registerCommand("L1", Robot.robotCommands.L1Command());
        NamedCommands.registerCommand("L2", Robot.robotCommands.L2Command());
        NamedCommands.registerCommand("L3", Robot.robotCommands.L3Command());
        NamedCommands.registerCommand("L4", Robot.robotCommands.L4AutoCommand());
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());
        NamedCommands.registerCommand("auto coral station align", Robot.robotCommands.autoCoralStationAlign());
        NamedCommands.registerCommand("auto reef align", Robot.robotCommands.autoReefAlign());
        NamedCommands.registerCommand("apply height cap", Robot.robotCommands.applyHeightCapCommand());
        NamedCommands.registerCommand("climb", Robot.robotCommands.climbCommand());
        NamedCommands.registerCommand("intake", Robot.robotCommands.intakeCommand());
        NamedCommands.registerCommand("inverted intake", Robot.robotCommands.invertedIntakeCommand());
        NamedCommands.registerCommand("home", Robot.robotCommands.homeCommand());
        NamedCommands.registerCommand("remove height cap", Robot.robotCommands.removeHeightCapCommand());

        autoChooser = AutoBuilder.buildAutoChooser();

        // DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

        //LimelightLocalization.getInstance();
        LED led = new LED(robotManager);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(autoChooser);
        if (alliance.isEmpty()) {
            alliance = DriverStation.getAlliance();
        }
        // blinkin.setColor(BlinkinColor
        }

    @Override
    public void disabledPeriodic() {
        alliance = DriverStation.getAlliance();
    }


    @Override
    public void teleopInit() {

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
        //ElevatorSubsystem.getInstance();
        if (autoChooser.getSelected() != null)
            autoChooser.getSelected().schedule();
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
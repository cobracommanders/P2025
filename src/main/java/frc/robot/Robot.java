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
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import java.util.List;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class Robot extends TimedRobot{
    public static final double DEFAULT_PERIOD = 0.02;
    public final Timer setupTimer = new Timer();
    public double setupTime  = 0;

    public static RobotManager robotManager = RobotManager.getInstance();
    public static RobotCommands robotCommands = new RobotCommands(robotManager);
    // public static int coordinateFlip = 1;
    // public static int rotationOffset = 0;

    public static Optional<Alliance> alliance = Optional.empty();
    public static final Controls controls = new Controls();

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    private SendableChooser<Command> autoChooser;


    @Override
    public void robotInit() {
        controls.configureDefaultCommands();
        controls.configureDriverCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        // Limelight.getInstance();
        
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
        
        
    if(RobotState.isEnabled()){
      //LED Code goes here
        }
      }


    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void disabledInit() {
        setupTimer.restart();
        // drivetrain.enableBrakeMode(false);
    }
    @Override
    public void autonomousInit() {
        // new FullScore().schedule();
        // drivetrain.enableBrakeMode(true);
        // matchStarted = true;
        ElevatorSubsystem.getInstance();

        // if (autoToRun == null)
            // autoToRun = defaultAuto;
        if (autoChooser.getSelected() != null)
            autoChooser.getSelected().schedule();
        //autoToRun = new HighHighCone();

        // if (alliance.get() == Alliance.Blue) {
        //     CommandSwerveDrivetrain.getInstance().(autoToRun.getInitialPose().getRotation().getDegrees());
        //     Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        // } else {
        //     Drivetrain.getInstance().setYaw(PoseUtil.flipAngleDegrees(autoToRun.getInitialPose().getRotation().getDegrees()));
        //     Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        // }
        //SmartDashboard.putData((Sendable) autoToRun.getInitialPose());

        // autoToRun.getCommand().schedule();
        //new LongTaxi().getCommand().schedule();

        // CommandScheduler.getInstance().run();

        // if (alliance.get() == Alliance.Blue) {
        //     Drivetrain.getInstance().setYaw(autoToRun.getInitialPose().getRotation().getDegrees());
        //     Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        // } else {
        //     Drivetrain.getInstance().setYaw(PoseUtil.flip(autoToRun.getInitialPose()).getRotation().getDegrees());
        //     Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        // }
        //Sets the LEDs to a pattern for auto, this could be edited to include code for if vision is aligned for auto diagnosis
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
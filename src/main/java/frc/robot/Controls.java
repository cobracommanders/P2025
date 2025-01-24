package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.RobotFlag;
import frc.robot.commands.RobotManager;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Controls {
    private  double MaxSpeed = TunerConstants.kSpeedAt12Volts; // Initial max is true top speed
    private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    private final double MaxAngularRate = Math.PI * 3.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

     SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(MaxSpeed * 0.1) // Deadband is handled on input
      .withRotationalDeadband(AngularRate * 0.1);

    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setDeadzone(0.15);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }
    private Supplier<SwerveRequest> controlStyle;
    private void newControlStyle () {
        controlStyle = () -> drive.withVelocityX(-driver.leftY() * driver.leftY() * driver.leftY() * MaxSpeed) // Drive forward -Y
            .withVelocityY(-driver.leftX() * driver.leftX() * driver.leftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(driver.rightX() * AngularRate); // Drive counterclockwise with negative X (left)
    }


     
    public void configureDefaultCommands() {
        newControlStyle();
         CommandSwerveDrivetrain.getInstance().setDefaultCommand(repeatingSequence( // Drivetrain will execute this command periodically
         runOnce(()-> CommandSwerveDrivetrain.getInstance().driveFieldRelative(new ChassisSpeeds(-driver.leftY() * driver.leftY() * driver.leftY() * MaxSpeed, -driver.leftX() * driver.leftX() * driver.leftX() * MaxSpeed, driver.rightX() * AngularRate)), CommandSwerveDrivetrain.getInstance())));
  }

    public void configureDriverCommands() {
        driver.rightBumper().onTrue(runOnce(() ->CommandSwerveDrivetrain.getInstance().setYaw(Robot.alliance.get())));
        driver.leftTrigger().onTrue(Robot.robotCommands.intakeCommand() );
            driver.leftTrigger().onFalse(Robot.robotCommands.idleCommand());
        driver.A().and(driver.leftTrigger()).onTrue(Robot.robotCommands.invertedIntakeCommand());
            // driver.A().and(driver.leftTrigger().onFalse(Robot.robotCommands.invertIdleCommand()));
    //    driver.B().onTrue(Commands.runOnce(()-> ElevatorSubsystem.getInstance().setStateFromRequest(ElevatorState.HOME_ELEVATOR), ElevatorSubsystem.getInstance()));
            driver.B().onFalse(Commands.runOnce(()-> ElevatorSubsystem.getInstance().setStateFromRequest(ElevatorState.IDLE), ElevatorSubsystem.getInstance()));
            driver.X().onTrue(Commands.runOnce(()-> ElbowSubsystem.getInstance().setStateFromRequest(ElbowState.HOME_ELBOW), ElbowSubsystem.getInstance()));
            driver.X().onFalse(Commands.runOnce(()-> ElbowSubsystem.getInstance().setStateFromRequest(ElbowState.IDLE), ElbowSubsystem.getInstance()));
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
            driver.rightTrigger().onFalse(Robot.robotCommands.idleCommand());
        driver.leftBumper().onTrue(Robot.robotCommands.removeHeightCapCommand());
            driver.leftBumper().onFalse(Robot.robotCommands.applyHeightCapCommand());
    }

    public void configureOperatorCommands(){
        operator.leftBumper().and(driver.A().negate()).onTrue(Robot.robotCommands.invertIdleCommand());
        //operator.leftBumper().onFalse(Robot.robotCommands.idleCommand());
        operator.rightBumper().onTrue(Robot.robotCommands.idleCommand());
        operator.Y().onTrue(Robot.robotCommands.L3Command());
        operator.B().onTrue(Robot.robotCommands.L4Command());
        operator.X().onTrue(Robot.robotCommands.L2Command());
        operator.A().onTrue(Robot.robotCommands.L1Command());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
        // operator.leftTrigger().and(operator.rightTrigger()).onFalse(Robot.robotCommands.idleCommand());
        //operator.POV90().onTrue(remove algea);
    }
}

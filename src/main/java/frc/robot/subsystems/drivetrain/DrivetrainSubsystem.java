package frc.robot.subsystems.drivetrain;

import java.security.InvalidParameterException;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.fasterxml.jackson.databind.node.NullNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.commands.RobotState;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.kicker.KickerState;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;

public class DrivetrainSubsystem extends StateMachine<DrivetrainState> {
  private  double MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final double MaxAngularRate = Math.PI * 3.5;
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
  private Xbox controller = Robot.controls.driver;


  public final Pigeon2 drivetrainPigeon = CommandSwerveDrivetrain.getInstance().getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private double goalSnapAngle = 0;

  public SwerveDriveState getDrivetrainState() {
    return drivetrainState;
  }

  public DrivetrainSubsystem() {
    super(DrivetrainState.DRIVE);
  }
  
  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
  }

    protected void afterTransition(DrivetrainState newState) {
      switch (newState) {
      case DRIVE -> {
      if (CommandSwerveDrivetrain.getInstance().isMoving()) {
        drivetrain.setControl(
              drive
              .withVelocityX(-controller.leftY() * controller.leftY() * controller.leftY() * MaxSpeed)
              .withVelocityY(-controller.leftX() * controller.leftX() * controller.leftX() * MaxSpeed)
              .withRotationalRate(controller.rightX() * MaxAngularRate)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      } else {
         drivetrain.applyRequest(() -> CommandSwerveDrivetrain.getInstance().brake);
      }
    }
      case CORAL_STATION_ALIGN ->
          drivetrain.setControl(
              driveToAngle
              .withVelocityX(-controller.leftY() * controller.leftY() * controller.leftY() * MaxSpeed)
              .withVelocityY(-controller.leftX() * controller.leftX() * controller.leftX() * MaxSpeed)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      case REEF_ALIGN ->
          drivetrain.setControl(
              driveToAngle
              .withVelocityX(-controller.leftY() * controller.leftY() * controller.leftY() * MaxSpeed)
              .withVelocityY(-controller.leftX() * controller.leftX() * controller.leftX() * MaxSpeed)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }
  }

  public void setState(DrivetrainState newState) {
    setStateFromRequest(newState);
  }

  @Override
  public void periodic() {
    super.periodic();

    }

    private static DrivetrainSubsystem instance;

    public static DrivetrainSubsystem getInstance() {
      if (instance == null) instance = new DrivetrainSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}

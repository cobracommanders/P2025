package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotState;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.util.PoseUtil;
import frc.robot.util.RobotPosition;
import frc.robot.vision.AlignmentState;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;

public class DrivetrainSubsystem extends StateMachine<DrivetrainState> {
  private  double MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();
  private final double MaxAngularRate = Math.PI * 4;
  private final CommandSwerveDrivetrain drivetrain;
  public Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
  public Pose2d nearestBranch = robotPose.nearest(List.of(LimelightLocalization.getInstance().branchPoses));
  public Pose2d nearestCoralStation = robotPose.nearest(List.of(LimelightLocalization.getInstance().coralStationPoses));
  private double goalSnapAngle = 0;


  private LimelightLocalization limelightLocalization = LimelightLocalization.getInstance();

  public final Pigeon2 drivetrainPigeon = CommandSwerveDrivetrain.getInstance().getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withTargetRateFeedforward(1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  
  private SwerveDriveState drivetrainState = new SwerveDriveState();

  public SwerveDriveState getDrivetrainState() {
    return drivetrainState;
  }

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;
  }
  public DrivetrainSubsystem() {
    super(DrivetrainState.TELEOP);
    drivetrain = CommandSwerveDrivetrain.getInstance();
    driveToAngle.HeadingController.setPID(6, 0, 0);
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.5);
  }

  @Override 
  protected DrivetrainState getNextState(DrivetrainState currentState) {
    DrivetrainState nextState = currentState;
     switch (currentState) {
      case TELEOP_CORAL_STATION_ALIGN, TELEOP_REEF_ALIGN -> {
        switch (RobotManager.getInstance().getState()) {
          case IDLE, INVERTED_IDLE, PREPARE_IDLE, PREPARE_INVERTED_IDLE, PREPARE_INVERTED_FROM_IDLE, PREPARE_IDLE_FROM_INVERTED-> {
            nextState = DrivetrainState.TELEOP;
          }
          default -> {}
        }
      }
      case TELEOP -> {
        switch (RobotManager.getInstance().getState()) {
          case PREPARE_CORAL_STATION, PREPARE_INVERTED_CORAL_STATION, INVERTED_INTAKE_CORAL_STATION, INTAKE_CORAL_STATION-> {
              nextState = DrivetrainState.TELEOP_CORAL_STATION_ALIGN;
          }
          case PREPARE_L1, PREPARE_L2, PREPARE_L3, PREPARE_L4, WAIT_L1, WAIT_L2, WAIT_L3, WAIT_L4, SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, CAPPED_L3, CAPPED_L4-> {
            nextState = DrivetrainState.TELEOP_REEF_ALIGN;
          }
          default -> {}
        }
      }
      default -> {}
     }
    return nextState;
  }
  

  boolean isControlled(ChassisSpeeds speeds) {
    return Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01 && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
  }

  @Override
  protected void collectInputs() {
    limelightLocalization.update();
    drivetrainState = drivetrain.getState();
    teleopSpeeds = new ChassisSpeeds(-Robot.controls.driver.leftY() * Robot.controls.driver.leftY() * Robot.controls.driver.leftY() * MaxSpeed, -Robot.controls.driver.leftX() * Robot.controls.driver.leftX() * Robot.controls.driver.leftX() * MaxSpeed, Robot.controls.driver.rightX() * MaxAngularRate);
    DogLog.log(getName() + "/Robot Heading", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees());
    DogLog.log(getName() + "/Robot Pose", CommandSwerveDrivetrain.getInstance().getState().Pose);
    boolean isSlow = false;
    if (!RobotManager.getInstance().isHeightCapped) {
      teleopSpeeds = teleopSpeeds.div(2);
      isSlow = true;
    }
    DogLog.log(getName() + "/isSlow", isSlow);
  }
  @Override
  public void periodic() {
    super.periodic();
    robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
    nearestBranch = robotPose.nearest(List.of(LimelightLocalization.getInstance().branchPoses));
    nearestCoralStation = robotPose.nearest(List.of(LimelightLocalization.getInstance().coralStationPoses));
    DogLog.log(getName() + "/Nearest Coral Station Pose", RobotPosition.calculateDegreesToCoralStation());
    sendSwerveRequest(getState());
  }

    protected void sendSwerveRequest(DrivetrainState newState) {
      switch (newState) {
      case TELEOP -> {
        drivetrain.setControl(
          drive
          .withVelocityX(teleopSpeeds.vxMetersPerSecond)
          .withVelocityY(teleopSpeeds.vyMetersPerSecond)
          .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      case TELEOP_CORAL_STATION_ALIGN -> {
          if (!isControlled(teleopSpeeds)) {
          drivetrain.setControl(
                driveToAngle
                .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                .withTargetDirection(Rotation2d.fromDegrees(RobotPosition.calculateDegreesToCoralStation()))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        } else {
          drivetrain.setControl(CommandSwerveDrivetrain.getInstance().brake);
        }
      }
      case TELEOP_REEF_ALIGN -> {
      if (!isControlled(teleopSpeeds)) {
        drivetrain.setControl(
              driveToAngle
              .withVelocityX(teleopSpeeds.vxMetersPerSecond)
              .withVelocityY(teleopSpeeds.vyMetersPerSecond)
              .withTargetDirection(Rotation2d.fromDegrees(RobotPosition.calculateDegreesToBranch()))
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      } else {
        drivetrain.setControl(CommandSwerveDrivetrain.getInstance().brake);
      }
    }
      case AUTO_CORAL_STATION_ALIGN ->
        {}
      case AUTO_REEF_ALIGN ->
        {}
      case AUTO ->
        {}
    }
  }

  public void setState(DrivetrainState newState) {
    setStateFromRequest(newState);
  }

    private static DrivetrainSubsystem instance;

    public static DrivetrainSubsystem getInstance() {
      if (instance == null) instance = new DrivetrainSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}

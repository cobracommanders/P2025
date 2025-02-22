package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.derive;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.config.PIDConstants;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.StateMachine;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotState;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.vision.AlignmentState;
import frc.robot.vision.AutoCoralAlignmentStates;
import frc.robot.vision.AutoReefAlignmentState;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;
import frc.robot.vision.LimelightState;
import frc.robot.vision.LimelightSubsystem;

public class DrivetrainSubsystem extends StateMachine<DrivetrainState> {
  private  double MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();
  private ChassisSpeeds reefAutoAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds coralStationAutoAlignSpeeds = new ChassisSpeeds();
  public PIDController coralStationAutoAlignTX = new PIDController(0, 0, 0);
  public PIDController coralStationAutoAlignTA = new PIDController(0, 0, 0);
  public PIDController reefAutoAlignTX = new PIDController(0, 0, 0);
  public PIDController reefAutoAlignTA = new PIDController(0, 0, 0);
  private final double MaxAngularRate = Math.PI * 3.5;
  public final CommandSwerveDrivetrain drivetrain;
  private LimelightLocalization limelightLocalization = LimelightLocalization.getInstance();
  private double snapCoralStationAngle;
  private double snapReefAngle;
  private int coralStationTag;
  private Pose2d targetCoralStationPose;

  public final Pigeon2 drivetrainPigeon = CommandSwerveDrivetrain.getInstance().getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

    public final SwerveRequest.RobotCentric driveRobotRelative =
          new SwerveRequest.RobotCentric()
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
    super(DrivetrainState.TELEOP);
    LimelightSubsystem.getInstance();
    drivetrain = CommandSwerveDrivetrain.getInstance();
    driveToAngle.HeadingController.setPID(6, 0, 0);
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.5);

    coralStationAutoAlignTA.setPID(0.3, 0, 0);
    coralStationAutoAlignTX.setPID(1.0 / 10.0, 0, 0);

    reefAutoAlignTA.setPID(0.1, 0, 0);
    reefAutoAlignTX.setPID(0.025, 0, 0);

    reefAutoAlignSpeeds = new ChassisSpeeds(0, 0, 0);
    coralStationAutoAlignSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  @Override
  protected DrivetrainState getNextState(DrivetrainState currentState) {
    DrivetrainState nextState = currentState;
     switch (currentState) {
      case AUTO_CORAL_STATION_ALIGN_1 -> {
        if (CommandSwerveDrivetrain.getInstance().isNear(targetCoralStationPose)) {
          nextState = DrivetrainState.AUTO_CORAL_STATION_ALIGN_2;
        }
      }
      case AUTO_CORAL_STATION_ALIGN_2 -> {
        if (LimelightLocalization.getInstance().getCoralStationAlignmentState(true) == AlignmentState.ALIGNED) {
          nextState = DriverStation.isAutonomous() ? DrivetrainState.AUTO : DrivetrainState.TELEOP; 
        }


      }
      case AUTO_REEF_ALIGN -> {
        if (LimelightLocalization.getInstance().getReefAlignmentState() == AlignmentState.ALIGNED) {
          nextState = DriverStation.isAutonomous() ? DrivetrainState.AUTO : DrivetrainState.TELEOP;
        }
      }
      case TELEOP_CORAL_STATION_ALIGN, TELEOP_REEF_ALIGN-> {
        switch (RobotManager.getInstance().getState()) {
          case IDLE, INVERTED_IDLE, PREPARE_IDLE, PREPARE_INVERTED_IDLE, PREPARE_INVERTED_FROM_IDLE, PREPARE_IDLE_FROM_INVERTED-> {
            nextState = DrivetrainState.TELEOP;
          }
          //  if(DriverStation.isAutonomous()){
          //   nextState = DrivetrainState.AUTO;
          //  }
          //   nextState = DrivetrainState.TELEOP;
          // }

          default -> {}
        }
      }
      case TELEOP-> {
        switch (RobotManager.getInstance().getState()) {
          case PREPARE_CORAL_STATION, PREPARE_INVERTED_CORAL_STATION, INVERTED_INTAKE_CORAL_STATION, INTAKE_CORAL_STATION-> {
            // nextState = DrivetrainState.TELEOP_CORAL_STATION_ALIGN;
            // if (DriverStation.isAutonomous()){
            //   nextState = DrivetrainState.AUTO_CORAL_STATION_ALIGN;
            // }
            // else nextState = DrivetrainState.TELEOP_CORAL_STATION_ALIGN;
            nextState = DrivetrainState.TELEOP_CORAL_STATION_ALIGN;
          }
          case PREPARE_L1, PREPARE_L2, PREPARE_L3, PREPARE_L4, WAIT_L1, WAIT_L2, WAIT_L3, WAIT_L4, SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, CAPPED_L4-> {
            // if (DriverStation.isAutonomous()){
            //   nextState = DrivetrainState.AUTO_REEF_ALIGN;
            // }
            // else nextState = DrivetrainState.TELEOP_REEF_ALIGN;
            nextState = DrivetrainState.TELEOP_REEF_ALIGN;
          }
          default -> {}
        }
      }
      default -> {}
     }
    return nextState;
  }

  public void setAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
  }
  
  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;
  }

  boolean isNotControlled(ChassisSpeeds speeds) {
    return Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01 && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
  }

  @Override
  protected void collectInputs() {
    limelightLocalization.update();
    drivetrainState = drivetrain.getState();
    teleopSpeeds = new ChassisSpeeds(
      -Robot.controls.driver.leftY() * Robot.controls.driver.leftY() * Robot.controls.driver.leftY() * MaxSpeed, 
      -Robot.controls.driver.leftX() * Robot.controls.driver.leftX() * Robot.controls.driver.leftX() * MaxSpeed, 
      Robot.controls.driver.rightX() * Robot.controls.driver.rightX() * Robot.controls.driver.rightX() * MaxAngularRate);

    DogLog.log(getName() + "/teleopSpeeds", teleopSpeeds);
    DogLog.log(getName() + "/robot pose", CommandSwerveDrivetrain.getInstance().getState().Pose);
    DogLog.log(getName() + "/coralStationTag", coralStationTag);
    boolean isSlow = false;
    if (!RobotManager.getInstance().isHeightCapped) {
      teleopSpeeds = teleopSpeeds.div(2);
      isSlow = true;
    }
    if (getState() == DrivetrainState.AUTO_CORAL_STATION_ALIGN_1 || getState() == DrivetrainState.AUTO_CORAL_STATION_ALIGN_2) {

      Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
      targetCoralStationPose = robotPose.nearest(List.of(LimelightLocalization.getInstance().coralStationPoses));
      snapCoralStationAngle = targetCoralStationPose.getRotation().getDegrees();//LimelightLocalization.getInstance().getCoralStationAngleFromTag();
      coralStationTag = (int) LimelightHelpers.getFiducialID("limelight-middle");
      boolean isValidTag = LimelightLocalization.coralStationTags.contains(coralStationTag);
      if (isValidTag) {
        double coralStationSpeedX = -coralStationAutoAlignTA.calculate(LimelightHelpers.getTA("limelight-middle"), 4.5);
        double coralStationSpeedY = coralStationAutoAlignTX.calculate(-LimelightHelpers.getTX("limelight-middle"), 0);
        coralStationAutoAlignSpeeds = new ChassisSpeeds(coralStationSpeedX, coralStationSpeedY, 0);
      } else {
        coralStationAutoAlignSpeeds = new ChassisSpeeds();
      }
    } 
    // else 
    if (getState() == DrivetrainState.AUTO_REEF_ALIGN) {
      double reefSpeedX = reefAutoAlignTA.calculate(LimelightHelpers.getTA("limelight-right"), 14);
      double reefSpeedY = reefAutoAlignTX.calculate(LimelightHelpers.getTX("limelight-right"), -14);
      reefAutoAlignSpeeds = new ChassisSpeeds(reefSpeedX, reefSpeedY, 0);
    }
    snapReefAngle = LimelightLocalization.getInstance().getReefAngleFromTag();
   
    DogLog.log(getName() + "/isSlow", isSlow);
    DogLog.log(getName() + "/Reef Snap Angle", snapReefAngle);
    DogLog.log(getName() + "/Coral Station Snap Angle", snapCoralStationAngle);
  }
  @Override
  public void periodic() {
    super.periodic();
    sendSwerveRequest(getState());
  }

    @Override
    protected void afterTransition(DrivetrainState newState) {
        switch (newState) {
          case TELEOP -> {
           LimelightSubsystem.getInstance().setState(LimelightState.DRIVE);
          }
          case AUTO -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO);
          }
          case TELEOP_REEF_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.REEF);
          }
          case TELEOP_CORAL_STATION_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.CORAL_STATION);
          }
          case AUTO_REEF_ALIGN -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO_REEF);
          }
          case AUTO_CORAL_STATION_ALIGN_1 -> {
            LimelightSubsystem.getInstance().setState(LimelightState.AUTO_CORAL_STATION);
          }
           default -> {}
        }
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
        drivetrain.setControl(
          drive
          .withVelocityX(teleopSpeeds.vxMetersPerSecond)
          .withVelocityY(teleopSpeeds.vyMetersPerSecond)
          .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      //Swerve X-ing code
        //   if (!isNotControlled(teleopSpeeds)) {
        //   drivetrain.setControl(
        //         drive
        //         .withVelocityX(teleopSpeeds.vxMetersPerSecond)
        //         .withVelocityY(teleopSpeeds.vyMetersPerSecond)
        //         .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
        //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        // } else {
        //   drivetrain.setControl(CommandSwerveDrivetrain.getInstance().brake);
      case TELEOP_REEF_ALIGN -> {
          drivetrain.setControl(
            drive
            .withVelocityX(teleopSpeeds.vxMetersPerSecond)
            .withVelocityY(teleopSpeeds.vyMetersPerSecond)
            .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
    //Swerve X-ing code
      // if (!isNotControlled(teleopSpeeds)) {
      //   drivetrain.setControl(
      //         drive
      //         .withVelocityX(teleopSpeeds.vxMetersPerSecond)
      //         .withVelocityY(teleopSpeeds.vyMetersPerSecond)
      //         .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
      //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      // } else {
      //   drivetrain.setControl(CommandSwerveDrivetrain.getInstance().brake);
      case AUTO_CORAL_STATION_ALIGN_1 -> {
        ChassisSpeeds speeds = drivetrain.driveToPoseSpeeds(targetCoralStationPose);
        drivetrain.setControl(

          driveToAngle
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withTargetDirection(targetCoralStationPose.getRotation())
            
        );
      }
      case AUTO_CORAL_STATION_ALIGN_2 -> {

        if (!MathUtil.isNear(snapCoralStationAngle, CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 3)) {
          drivetrain.setControl(
            driveToAngle
              .withVelocityX(autoSpeeds.vxMetersPerSecond)
              .withVelocityY(autoSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withTargetDirection(Rotation2d.fromDegrees(snapCoralStationAngle)));
        } else {
          drivetrain.setControl(
            driveRobotRelative
              .withVelocityX(coralStationAutoAlignSpeeds.vxMetersPerSecond)
              .withVelocityY(coralStationAutoAlignSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.Velocity));
        }
      }
        // if (coralStationTag == 12) {
        //   if (!MathUtil.isNear(snapCoralStationAngle, CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 1)) {
        //       drivetrain.setControl(
                
        //         driveToAngle
        //           .withVelocityX(autoSpeeds.vxMetersPerSecond)
        //           .withVelocityY(autoSpeeds.vyMetersPerSecond)
        //           .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        //           .withTargetDirection(Rotation2d.fromDegrees(snapCoralStationAngle)));
        //     }
        //   else {ChassisSpeeds speeds = drivetrain.driveToPoseSpeeds(LimelightLocalization.tag12Pose);
        //   drivetrain.setControl(
        //     new SwerveRequest.FieldCentricFacingAngle()
        //       .withVelocityX(speeds.vxMetersPerSecond)
        //       .withVelocityY(speeds.vyMetersPerSecond)
        //       .withTargetDirection(Rotation2d.fromDegrees(snapCoralStationAngle))
        //       .withDeadband(MaxSpeed * 0.03)
        //       .withDriveRequestType(DriveRequestType.Velocity)
        //   );}}

        // }
        // if (!MathUtil.isNear(snapCoralStationAngle, CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 3)) {
        //   drivetrain.setControl(
        //     driveToAngle
        //       .withVelocityX(autoSpeeds.vxMetersPerSecond)
        //       .withVelocityY(autoSpeeds.vyMetersPerSecond)
        //       .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        //       .withTargetDirection(Rotation2d.fromDegrees(snapCoralStationAngle)));
        // } else {
        //   drivetrain.setControl(
        //     driveRobotRelative
        //       .withVelocityX(0)
        //       .withVelocityY(coralStationAutoAlignSpeeds.vyMetersPerSecond)
        //       .withDriveRequestType(DriveRequestType.Velocity));
        // }
        
      case AUTO_REEF_ALIGN -> {
        if (!MathUtil.isNear(snapReefAngle, CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 3)) {
          drivetrain.setControl(
            driveToAngle
              .withVelocityX(autoSpeeds.vxMetersPerSecond)
              .withVelocityY(autoSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withTargetDirection(Rotation2d.fromDegrees(snapReefAngle)));
        } else {
          drivetrain.setControl(
            driveRobotRelative
              .withVelocityX(reefAutoAlignSpeeds.vxMetersPerSecond)
              .withVelocityY(reefAutoAlignSpeeds.vyMetersPerSecond)
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case AUTO -> {
        drivetrain.setControl(
          driveRobotRelative
            .withVelocityX(autoSpeeds.vxMetersPerSecond)
            .withVelocityY(autoSpeeds.vyMetersPerSecond)
            .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
            .withDriveRequestType(DriveRequestType.Velocity)
        );
      }
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

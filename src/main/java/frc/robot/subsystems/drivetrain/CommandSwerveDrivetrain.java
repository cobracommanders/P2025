package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field = new Field2d();
    public SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private PIDController xController = new PIDController(5.0, 0, 0);
    private PIDController yController = new PIDController(5.0, 0, 0);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        
        AutoBuilder.configure(
            () -> this.getState().Pose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> DrivetrainSubsystem.getInstance().setAutoSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds driveToPoseSpeeds(Pose2d pose) {
        double xSpeed = xController.calculate(getState().Pose.getX(), pose.getX());
        double ySpeed = yController.calculate(getState().Pose.getY(), pose.getY());
        return new ChassisSpeeds(xSpeed, ySpeed, 0);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {

            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setYaw(Rotation2d rotation) {
        this.resetRotation(rotation);
    }
    
    public void setYaw(Alliance alliance) { 
        // setYaw(Rotation2d.fromDegrees(0));
        setYaw((alliance == Alliance.Red) ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation); 
    }

    public boolean isMoving() {
        return (Math.abs(this.getState().Speeds.vxMetersPerSecond) >= 1 || Math.abs(this.getState().Speeds.vyMetersPerSecond) >= 1 || Math.abs(this.getState().Speeds.omegaRadiansPerSecond) >= 0.5);
    }

    public boolean isNear(Pose2d pose) {
        Pose2d robotPose = getState().Pose;
        return MathUtil.isNear(robotPose.getX(), pose.getX(), 0.03) && MathUtil.isNear(robotPose.getY(), pose.getY(), 0.03);
    }

    public boolean isNear(Pose2d pose, double tolerance) {
        Pose2d robotPose = getState().Pose;
        return MathUtil.isNear(robotPose.getX(), pose.getX(), tolerance) && MathUtil.isNear(robotPose.getY(), pose.getY(), tolerance);
    }

    public void update() {
        field.setRobotPose(this.getState().Pose);
        SmartDashboard.putData(field);
        DogLog.log("heading", this.getState().Pose.getRotation().getDegrees());
        DogLog.log("robot speed", this.getState().Speeds.vxMetersPerSecond);
        DogLog.log("robot speed", this.getState().Speeds.vyMetersPerSecond);
        DogLog.log("has applied operator prespective", hasAppliedOperatorPerspective);
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
    
    private static CommandSwerveDrivetrain instance;

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null) instance = TunerConstants.createDrivetrain(); 
        return instance;

    }
}

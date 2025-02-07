package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightLocalization;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotPosition {
    public static double calculateDegreesToTarget(Pose2d target) {
        Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        double currentvxSpeed = CommandSwerveDrivetrain.getInstance().getState().Speeds.vxMetersPerSecond;
        double currentvySpeed = CommandSwerveDrivetrain.getInstance().getState().Speeds.vyMetersPerSecond;

        // Estimate the future pose of the robot to compensate for lag
        double newX;
        double newY;
        if (Robot.alliance.get() == Alliance.Red) {
            newX = currentPose.getX() + (-currentvxSpeed);
            newY = currentPose.getY() + (-currentvySpeed);
        } else {
            newX = currentPose.getX() + (currentvxSpeed);
            newY = currentPose.getY() + (currentvySpeed);
        }

        Pose2d futurePose = new Pose2d(newX, newY, new Rotation2d());

        // Calculate the angle between the target and the current robot position.
        double angle = Math.toDegrees(Math.atan2(-futurePose.getY() + target.getY(), -futurePose.getX() + target.getX()));

        // Normalize the angle to a number between 0 and 360.
        angle = RotationUtil.toSignedDegrees(angle);

        // Return the angle to which the turret needs to be adjusted.
        return angle;
    }

    public static double calculateDegreesToCoralStation() {
        Pose2d closestCoralStation = DrivetrainSubsystem.getInstance().nearestCoralStation;
        if (Robot.alliance.get() == Alliance.Red) return calculateDegreesToTarget(PoseUtil.flip(closestCoralStation));
        return calculateDegreesToTarget(closestCoralStation);
    }
    public static double calculateDegreesToBranch() {
        Pose2d closestBranch = DrivetrainSubsystem.getInstance().nearestCoralStation;
        if (Robot.alliance.get() == Alliance.Red) return calculateDegreesToTarget(PoseUtil.flip(closestBranch));
        return calculateDegreesToTarget(closestBranch);
    }
}

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RotationUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotPosition {
    public static double calculateDegreesToTarget(Pose2d target, double tof) {
        Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds();

        // Estimate the future pose of the robot to compensate for lag
        double newX;
        double newY;
        if (Robot.alliance.get() == Alliance.Red) {
            newX = currentPose.getX() + (-currentSpeeds.vxMetersPerSecond * tof);
            newY = currentPose.getY() + (-currentSpeeds.vyMetersPerSecond * tof);
        } else {
            newX = currentPose.getX() + (currentSpeeds.vxMetersPerSecond * tof);
            newY = currentPose.getY() + (currentSpeeds.vyMetersPerSecond * tof);
        }
        

        Pose2d futurePose = new Pose2d(newX, newY, new Rotation2d());

        // Calculate the angle between the target and the current robot position.
        double angle = Math.toDegrees(Math.atan2(-futurePose.getY() + target.getY(), -futurePose.getX() + target.getX()));

        // Normalize the angle to a number between 0 and 360.
        angle = RotationUtil.toSignedDegrees(angle);

        // Return the angle to which the turret needs to be adjusted.
        return angle;
    }
    public static double calculateDegreesToTarget(Pose2d target) {
        return calculateDegreesToTarget(target, defaultTOF);
    }
}

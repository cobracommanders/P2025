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
    public static double calculateDegreesToCoralStation() {
        Pose2d closestCoralStation = DrivetrainSubsystem.getInstance().nearestCoralStation;
        double coralStationFace = closestCoralStation.getRotation().getDegrees();
        return coralStationFace;
    }
    
    public static double calculateDegreesToBranch() {
        Pose2d closestBranch = DrivetrainSubsystem.getInstance().nearestBranch;
        double branchFace = closestBranch.getRotation().getDegrees();
        return branchFace;
    }
}

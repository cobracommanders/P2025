package frc.robot;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.vision.LimelightLocalization;


public final class FieldConstants {
  public Pose2d closestBranch;
  public Pose2d closestCoralStation;
  public Pose2d closestAlgae;
  
  public Pose2d[] branchPosesBlue = {
    new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(-60)), //L
    new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(-60)), //K
    new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(-120)), //J
    new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(-120)), //I
    new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)), //H
    new Pose2d(5.27, 3.86, Rotation2d.fromDegrees(180)), //G
    new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)), //F
    new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)), //E
    new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)), //D
    new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)), //C
    new Pose2d(3.71, 3.86, Rotation2d.fromDegrees(0)), //B
    new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)), //A
  };
  public Pose2d[] branchPosesRed = {
    new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)), //L
    new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120)), //K
    new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60)), //J
    new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60)), //I
    new Pose2d(12.29, 3.86, Rotation2d.fromDegrees(0)), //H
    new Pose2d(12.29, 4.19, Rotation2d.fromDegrees(0)), //G
    new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(-60)), //F
    new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(-60)), //E
    new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(-120)), //D
    new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(-120)), //C
    new Pose2d(13.84, 4.19, Rotation2d.fromDegrees(180)), //B
    new Pose2d(13.84, 3.86, Rotation2d.fromDegrees(180)), //A
  };

  public Pose2d[] algaePosesBlue = {
    new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(-60)), //L + K low
    new Pose2d(4.905, 4.745, Rotation2d.fromDegrees(-120)), //J + I high
    new Pose2d(5.321, 4.026, Rotation2d.fromDegrees(180)), //H + G low
    new Pose2d(4.905, 3.307, Rotation2d.fromDegrees(120)), //F + E high
    new Pose2d(4.074, 3.307, Rotation2d.fromDegrees(60)), //D + C low
    new Pose2d(3.658, 4.026, Rotation2d.fromDegrees(0)), //B + A high
  };

  public Pose2d[] algaePosesRed = {
    new Pose2d(13.47, 3.307, Rotation2d.fromDegrees(120)), //L + K low
    new Pose2d(12.643, 3.307, Rotation2d.fromDegrees(60)), //J + I high
    new Pose2d(12.227, 4.026, Rotation2d.fromDegrees(0)), //H + G low
    new Pose2d(12.643, 4.745, Rotation2d.fromDegrees(-60)), //F + E high
    new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(-120)), //D + C low
    new Pose2d(13.884, 4.026, Rotation2d.fromDegrees(180)), //B + A high
  };

  // public Pose2d[] lowAlgaePoses = {
  //   new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(-60)), //L + K blue
  //   new Pose2d(5.321, 4.026, Rotation2d.fromDegrees(180)), //H + G blue
  //   new Pose2d(4.074, 3.307, Rotation2d.fromDegrees(60)), //D + C blue
  //   new Pose2d(13.47, 3.307, Rotation2d.fromDegrees(120)), //L + K red 
  //   new Pose2d(12.227, 4.026, Rotation2d.fromDegrees(0)), //H + G red
  //   new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(-120)), //D + C red
  // };
  // public Pose2d[] highAlgaePoses = {
  //   new Pose2d(4.905, 4.745, Rotation2d.fromDegrees(-120)), //J + I blue
  //   new Pose2d(4.905, 3.307, Rotation2d.fromDegrees(120)), //F + E blue
  //   new Pose2d(3.658, 4.026, Rotation2d.fromDegrees(0)), //B + A blue
  //   new Pose2d(12.643, 3.307, Rotation2d.fromDegrees(60)), //J + I red
  //   new Pose2d(12.643, 4.745, Rotation2d.fromDegrees(-60)), //F + E red 
  //   new Pose2d(13.884, 4.026, Rotation2d.fromDegrees(180)), //B + A red
  // };

  public Pose2d[] coralStationPosesBlue = {
    // new Pose2d(1.129, 7.122, Rotation2d.fromDegrees(-50)), // tag 13 CS
    // new Pose2d(1.199, 0.974, Rotation2d.fromDegrees(50)) // tag 12 CS
    new Pose2d(0.8512, 7.396, Rotation2d.fromDegrees(-50)), // tag 13 CS
    new Pose2d(0.8512, 0.655, Rotation2d.fromDegrees(50)) // tag 12 CS
  };
  public Pose2d[] coralStationPosesRed = {
    // new Pose2d(16.46, 0.93, Rotation2d.fromDegrees(130)), // tag 1 CS
    // new Pose2d(16.40, 7.028, Rotation2d.fromDegrees(-130)) // tag 2 CS
    new Pose2d(16.697, 0.655, Rotation2d.fromDegrees(130)), // tag 1 CS
    new Pose2d(16.697, 7.396, Rotation2d.fromDegrees(-130)) // tag 2 CS
  };

  public double bargeCoordinate = 8.775;


  public Pose2d[] getCoralStationPoses() {
    return Robot.alliance.get() == Alliance.Red ? coralStationPosesRed : coralStationPosesBlue;
  }
  public Pose2d[] getReefPoses(){
    return Robot.alliance.get() == Alliance.Red ? branchPosesRed : branchPosesBlue;
  }
  public Pose2d[] getAlgaePoses() {
    return Robot.alliance.get() == Alliance.Red ? algaePosesRed : algaePosesBlue;
  }

  public Pose2d getNearestBranch() {
      closestBranch = DrivetrainSubsystem.getInstance().drivetrain.getState().Pose.nearest(List.of(getReefPoses()));
      return closestBranch;
  }
  public Pose2d getNearestCoralStation() {
      closestCoralStation = DrivetrainSubsystem.getInstance().drivetrain.getState().Pose.nearest(List.of(getCoralStationPoses()));
      return closestCoralStation;
  }
  public Pose2d getNearestAlgae() {
    closestAlgae = DrivetrainSubsystem.getInstance().drivetrain.getState().Pose.nearest(List.of(getAlgaePoses()));
    return closestAlgae;
  }

  public boolean isNearHighAlgae() {
    if (getNearestAlgae() == algaePosesBlue[1] || getNearestAlgae() == algaePosesBlue[3] || getNearestAlgae() == algaePosesBlue[5] ||
        getNearestAlgae() == algaePosesRed[1] || getNearestAlgae() == algaePosesRed[3] || getNearestAlgae() == algaePosesRed[5]) {
      return true;
    } else {
      return false;
    }
  }

  public void logBranches() {
    int i = 0;
    for (Pose2d pose : branchPosesRed) {
      i++;
      DogLog.log("FieldConstants/branch pose " + i, pose);
      Pose2d offsetPose = LimelightLocalization.getInstance().getAdjustedBranchPose(pose);
      DogLog.log("FieldConstants/branch pose offset " + i, offsetPose);
    }
  }

  public void logAlgae() {
    int i = 0;
    for (Pose2d pose : algaePosesRed) {
      i++;
      DogLog.log("FieldConstants/algae pose " + i, pose);
      Pose2d offsetPose = LimelightLocalization.getInstance().getAdjustedAlgaePose(pose);
      DogLog.log("FieldConstants/algae pose offset " + i, offsetPose);
    }
  }

  public void logCoralStation() {
    int i = 0;
    for (Pose2d pose : coralStationPosesRed) {
      i++;
      DogLog.log("FieldConstants/coral station id pose " + i, pose);
      Pose2d offsetPose = LimelightLocalization.getInstance().getAdjustedCoralStationPose(pose);
      DogLog.log("FieldConstants/coral station pose offset " + i, offsetPose);
    }
  }

  private static FieldConstants instance;
  
  public static FieldConstants getInstance() {
      if (instance == null) instance = new FieldConstants(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}

package frc.robot.vision;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;


public class LimelightLocalization{
  private boolean rejectLeftData;
  private boolean rejectRightData;
  public LimelightLocalization() {

  }

  public void update(){
    rejectLeftData = false;
    rejectRightData = false;
    
    LimelightHelpers.SetRobotOrientation("limelight-left", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2l = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate mt2r = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    if(CommandSwerveDrivetrain.getInstance().isMoving() && DriverStation.isTeleop()) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      rejectLeftData = true;
      rejectRightData = true;
    }
    if(mt2r == null || mt2r.tagCount == 0)
    {
      rejectRightData = true;
    }
    if(mt2l == null || mt2l.tagCount == 0)// || DriverStation.isAutonomous())
    {
      rejectLeftData = true;
    }
    if(!rejectRightData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2r.pose,
          Utils.fpgaToCurrentTime(mt2r.timestampSeconds),
          VecBuilder.fill(0.75,0.75,9999999));
      SmartDashboard.putNumber("mt2r", mt2r.timestampSeconds);

    }
    if(!rejectLeftData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2l.pose,
          mt2l.timestampSeconds,
          VecBuilder.fill(0.75, 0.75,9999999));
          SmartDashboard.putNumber("mt2l", mt2l.timestampSeconds);

    }
    
  }
}

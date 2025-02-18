package frc.robot.vision;
import java.util.List;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;


public class LimelightLocalization{
  public boolean rejectLeftData;
  public boolean rejectRightData;
  public boolean rejectMiddleData;
  public boolean disableLeft;
  public boolean disableRight;
  public boolean disableMiddle;
  public double limelightTX;
  public double limelightTXsetpoint;

  public Pose2d[] branchPoses = {
    new Pose2d(5, 5.247, Rotation2d.fromDegrees(-120)), //J
    new Pose2d(5.285, 5.107, Rotation2d.fromDegrees(-120)), //I
    new Pose2d(5.843, 4.2, Rotation2d.fromDegrees(180)), //H
    new Pose2d(5.843, 3.841, Rotation2d.fromDegrees(180)), //G
    new Pose2d(5.295, 2.923, Rotation2d.fromDegrees(120)), //F
    new Pose2d(5.036, 2.784, Rotation2d.fromDegrees(120)), //E
    new Pose2d(3.949, 2.774, Rotation2d.fromDegrees(60)), //D
    new Pose2d(3.65, 2.943, Rotation2d.fromDegrees(60)), //C
    new Pose2d(3.171, 3.841, Rotation2d.fromDegrees(0)), //B
    new Pose2d(3.171, 4.170, Rotation2d.fromDegrees(0)), //A
    new Pose2d(3.719, 5.087, Rotation2d.fromDegrees(-60)), //L
    new Pose2d(4, 5.227, Rotation2d.fromDegrees(-60)), //K
  };
  public Pose2d[] coralStationPoses = {
    new Pose2d(1.227, 7.071, Rotation2d.fromDegrees(120)), // Left CS
    new Pose2d(7.936, 5.817, Rotation2d.fromDegrees(-120)) // Right CS
  };
  public LimelightLocalization() {
    LimelightHelpers.setPipelineIndex("limelight-left", 2);
    LimelightHelpers.setPipelineIndex("limelight-right", 2);
    LimelightHelpers.setPipelineIndex("limelight-middle", 2);
  }

  public AutoReefAlignmentState getCoralAutoAlignmentStates(){
    double tolerance = 3;

    if(Math.abs(LimelightHelpers.getTA("limelight-middle")) > 4.5){
      return AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX;
    }

    if(Math.abs(LimelightHelpers.getTX("limelight-middle") + 5) < tolerance && getCoralAutoAlignmentStates() == AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX){
      return AutoReefAlignmentState.AUTO_ALIGNED_TX;
    }

    if (LimelightHelpers.getCurrentPipelineIndex("limelight-left") != 2 || LimelightHelpers.getCurrentPipelineIndex("limelight-middle") != 2){
      return AutoReefAlignmentState.INVALID;
    }

    else {
      return AutoReefAlignmentState.NOT_ALIGNED;
    }

  }

  public AutoReefAlignmentState getReefAutoAlignmentStates(){
    double tolerance = 4;

    if(Math.abs(LimelightHelpers.getTA("limelight-left")) > Math.abs(LimelightHelpers.getTA("limelight-right"))){
      limelightTX = LimelightHelpers.getTX("limelight-left");
      limelightTXsetpoint = -6;
      return AutoReefAlignmentState.AUTO_NOT_ALIGNED_TA;
    }

    if(Math.abs(LimelightHelpers.getTA("limelight-right")) > Math.abs(LimelightHelpers.getTA("limelight-left"))){
      limelightTX = LimelightHelpers.getTX("limelight-right");
      limelightTXsetpoint = 18;
      return AutoReefAlignmentState.AUTO_NOT_ALIGNED_TA;
    }

    if(Math.abs(LimelightHelpers.getTA("limelight-right")) > 13){
      return AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX;
    }

    if(Math.abs(LimelightHelpers.getTA("limelight-left")) > 13){
      return AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX;
    }

    if(Math.abs(LimelightHelpers.getTX("limelight-right") + 18) < tolerance && getReefAutoAlignmentStates() == AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX){
      return AutoReefAlignmentState.AUTO_ALIGNED_TX;
    }

    if(Math.abs(LimelightHelpers.getTX("limelight-left") - 6) < tolerance && getReefAutoAlignmentStates() == AutoReefAlignmentState.AUTO_NOT_ALIGNED_TX){
      return AutoReefAlignmentState.AUTO_ALIGNED_TX;
    }

    if (LimelightHelpers.getCurrentPipelineIndex("limelight-left") != 2 || LimelightHelpers.getCurrentPipelineIndex("limelight-right") != 2){
      return AutoReefAlignmentState.INVALID;
    }

    else {
      return AutoReefAlignmentState.NOT_ALIGNED;
    }

  }

  public AlignmentState getReefAlignmentState(){

    double tolerance = 4;

    // Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
    // Pose2d nearestBranch = robotPose.nearest(List.of(branchPoses));
    //Transform2d poseDifference = nearestBranch.minus(robotPose);
    if ((Math.abs(LimelightHelpers.getTX("limelight-left") - 6)  < tolerance && LimelightHelpers.getTA("limelight-left") > 10) || 
      (Math.abs(LimelightHelpers.getTX("limelight-right") + 14) < tolerance && LimelightHelpers.getTA("limelight-right") > 10)) {
      return AlignmentState.ALIGNED;
    }
    // else if (Math.abs(LimelightHelpers.getTX("limelight-left") - 6)  < tolerance || Math.abs(LimelightHelpers.getTX("limelight-right") + 8) < tolerance){
    //   return AlignmentState.FAR_RIGHT;
    // }
    else {
      return AlignmentState.NOT_ALIGNED;
    }

  }
  
  public double getCoralStationAngleFromTag() {
    switch ((int) LimelightHelpers.getFiducialID("limelight-middle")) {
      case 13:
        return -50;
      case 12:
        return 50;
      case 2:
        return -130;
      case 1:
        return 130;
      default:
        return 0;
    }
  }

  public double getReefAngleFromTag() {
    switch ((int) LimelightHelpers.getFiducialID("limelight-middle")) {
      case 6:
        return 120;
      case 7:
        return 180;
      case 8:
        return -120;
      case 9:
        return -60;
      case 10:
        return 0;
      case 11:
        return 60;
      case 17:
        return 60;
      case 18:
        return 0;
      case 19:
        return -60;
      case 20:
        return -120;
      case 21:
        return 180;
      case 22:
        return 120;
      default:
        return 0;
    }
  }

  public AlignmentState getCoralStationAlignmentState(){
    double tolerance = 3;
    if (LimelightHelpers.getCurrentPipelineIndex("limelight-middle") != 2){
      return AlignmentState.INVALID;
    }
    // Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
    // Pose2d nearestCoralStation = robotPose.nearest(List.of(coralStationPoses));
    // Transform2d poseDifference = nearestCoralStation.minus(robotPose);
    if (Math.abs(LimelightHelpers.getTX("limelight-middle")) < tolerance && LimelightHelpers.getTA("limelight-middle") > 3.5) {
      return AlignmentState.ALIGNED;
    }
    // else if (poseDifference.getX() < -tolerance){
    //   return AlignmentState.FAR_RIGHT;
    // }
    else{
      return AlignmentState.NOT_ALIGNED;
    }
  }

  public void update(){
    rejectLeftData = false;
    rejectRightData = false;
    rejectMiddleData = false;
    
    LimelightHelpers.SetRobotOrientation("limelight-left", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-middle", CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2l = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate mt2r = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    LimelightHelpers.PoseEstimate mt2m = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-middle");
    if(CommandSwerveDrivetrain.getInstance().isMoving() && DriverStation.isTeleop()) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      rejectLeftData = true;
      rejectRightData = true;
      rejectMiddleData = true;
    }
    if(mt2m == null || mt2m.tagCount == 0 || disableMiddle)
    {
      rejectMiddleData = true;
    }
    if(mt2r == null || mt2r.tagCount == 0 || disableRight)
    {
      rejectRightData = true;
    }
    if(mt2l == null || mt2l.tagCount == 0 || disableLeft)// || DriverStation.isAutonomous())
    {
      rejectLeftData = true;
    }
    // if (DriverStation.isAutonomous()){
    //   rejectLeftData = true;
    //   rejectRightData = true;
    //   rejectMiddleData = true;
    // }
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
    if(!rejectMiddleData)
    {

      CommandSwerveDrivetrain.getInstance().addVisionMeasurement(
          mt2m.pose,
          mt2m.timestampSeconds,
          VecBuilder.fill(0.75, 0.75,9999999));
          SmartDashboard.putNumber("mt2m", mt2m.timestampSeconds);

    }

    
    if(Math.abs(LimelightHelpers.getTA("limelight-left")) > Math.abs(LimelightHelpers.getTA("limelight-right"))){
      rejectRightData = true;
      rejectLeftData = false;
      rejectMiddleData = true;
    }

    if(Math.abs(LimelightHelpers.getTA("limelight-right")) > Math.abs(LimelightHelpers.getTA("limelight-left"))){
      rejectLeftData = true;
      rejectRightData = false;
      rejectMiddleData = true;
    }

    if(Math.abs(LimelightHelpers.getTA("limelight-middle")) > Math.abs(LimelightHelpers.getTA("limelight-left")) || Math.abs(LimelightHelpers.getTA("limelight-middle")) > Math.abs(LimelightHelpers.getTA("limelight-right")) ){
      rejectLeftData = true;
      rejectRightData = true;
      rejectMiddleData = false;
    }
    

  }

  public void updateLeftCamera() {

  }

  private static LimelightLocalization instance;

    public static LimelightLocalization getInstance(){
       if (instance == null) instance = new LimelightLocalization(); // Make sure there is an instance (this will only run once)
        return instance;
    }

}

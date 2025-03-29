package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.StateMachine;



public class LimelightSubsystem extends StateMachine<LimelightState>{
   NetworkTable leftLimelightTable = NetworkTableInstance.getDefault().getTable("left-limelight");
   NetworkTable rightLimelightTable = NetworkTableInstance.getDefault().getTable("right-limelight");
   NetworkTable middleLimelightTable = NetworkTableInstance.getDefault().getTable("middle-limelight");
  

  private LimelightLocalization limelightLocalization = LimelightLocalization.getInstance();

  public LimelightSubsystem() {
     super(LimelightState.DRIVE);
     limelightLocalization.disableLeft = false;
     limelightLocalization.disableRight = false;
     limelightLocalization.disableMiddle = false;
     LimelightHelpers.setPipelineIndex("limelight-left", 0);
     LimelightHelpers.setPipelineIndex("limelight-left", 0);
     LimelightHelpers.setPipelineIndex("limelight-left", 0);
    }

    protected LimelightState getNextState(LimelightState currentState) {
      return currentState;
  }

    @Override
    public void collectInputs(){
      limelightLocalization.collectInputs();
        DogLog.log(getName() + "/left camera reject data", limelightLocalization.rejectLeftData);
        DogLog.log(getName() + "/right camera reject data", limelightLocalization.rejectRightData);
        DogLog.log(getName() + "/middle camera reject data", limelightLocalization.rejectMiddleData);
       }
    
    public void setState(LimelightState newState) {
        setStateFromRequest(newState);
    }
  
      @Override
      protected void afterTransition(LimelightState newState) {
        switch (newState) {
          case REEF -> {
            limelightLocalization.disableLeft = false;
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = true;
          }
          case CORAL_STATION -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight = true;
            limelightLocalization.disableMiddle = false;
          }
          case DRIVE -> {
            limelightLocalization.disableLeft = false;
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = false;
          }
          case DISABLED -> {
            limelightLocalization.disableLeft = false;
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = true;
          }
          case AUTO -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight = true;
            limelightLocalization.disableMiddle = true;
          }
          case AUTO_CORAL_STATION -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight = true;
            limelightLocalization.disableMiddle = false;
          }
          case AUTO_REEF -> {
            limelightLocalization.disableLeft = false;
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = true;
          }
          case BARGE_ALIGN -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight = true;
            limelightLocalization.disableMiddle = false;
          }
          default -> {}
          }

      }

      
  
    private static LimelightSubsystem instance;
  
    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
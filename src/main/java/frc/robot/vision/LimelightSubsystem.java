package frc.robot.vision;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elbow.ElbowState;



public class LimelightSubsystem extends StateMachine<LimelightState>{
  

  private LimelightLocalization limelightLocalization = LimelightLocalization.getInstance();

  public LimelightSubsystem() {
     super(LimelightState.DRIVE);
    }

    protected LimelightState getNextState(LimelightState currentState) {
      return currentState;
  }

    @Override
    public void collectInputs(){
    }
    
    public void setState(LimelightState newState) {
        setStateFromRequest(newState);
    }
  
      @Override
      protected void afterTransition(LimelightState newState) {
        switch (newState) {
          case REEF -> {
            limelightLocalization.disableLeft = false;
            LimelightHelpers.setPipelineIndex("limelight-left", 2);
            LimelightHelpers.setPipelineIndex("limelight-right", 2);
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = true;
          }
          case CORAL_STATION -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight =true;
            LimelightHelpers.setPipelineIndex("limelight-middle", 2);
            limelightLocalization.disableMiddle = false;
          }
          case DRIVE -> {
            limelightLocalization.disableLeft = false;
            limelightLocalization.disableRight = false;
            limelightLocalization.disableMiddle = false;
            LimelightHelpers.setPipelineIndex("limelight-left", 0);
            LimelightHelpers.setPipelineIndex("limelight-right", 0);
            LimelightHelpers.setPipelineIndex("limelight-middle", 0);
          }
          case DISABLED -> {
            limelightLocalization.disableLeft = true;
            limelightLocalization.disableRight = true;
            limelightLocalization.disableMiddle = true;
            LimelightHelpers.setPipelineIndex("limelight-left", 0);
            LimelightHelpers.setPipelineIndex("limelight-right", 0);
            LimelightHelpers.setPipelineIndex("limelight-middle", 0);
          }
          default -> {}
        }

        DogLog.log(getName() + "/left camera disabled", limelightLocalization.disableLeft);
        DogLog.log(getName() + "/right camera disabled", limelightLocalization.disableRight);
        DogLog.log(getName() + "/middle camera disabled", limelightLocalization.disableMiddle);
      }
  
    private static LimelightSubsystem instance;
  
    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }
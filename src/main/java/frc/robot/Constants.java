package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class Constants {
  public static final class DrivetrainConstants{
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;
  }

public static final class ElevatorConstants{
  public static final double homingStallCurrent = 30;
  public static final double P = 40;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.5075;
  public static final double MotionMagicAcceleration = 40; //40
  public static final double MotionMagicCruiseVelocity = 200; //115
  public static final double MotionMagicJerk = 250; //175
}

public static final class WristConstants{
  public static final double homingStallCurrent = 10;
  public static final double P = 28;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.15;
  public static final double MotionMagicAcceleration = 11;
  public static final double MotionMagicCruiseVelocity = 40;
  public static final double MotionMagicJerk = 95;
}
public static final class ElbowConstants{
  public static final double homingStallCurrent = 15;
  public static final double P = 40;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0.2;
  public static final double MotionMagicAcceleration = 16;
  public static final double MotionMagicCruiseVelocity = 50;
  public static final double MotionMagicJerk = 150;
}

public static final class ManipulatorConstants{
  public static final double coralStallCurrent = 5;
}


public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
}
  }


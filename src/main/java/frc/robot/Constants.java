package frc.robot;

public final class Constants {
  public static final class DrivetrainConstants{
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;
  }

public static final class ElevatorConstants{
  public static final double homingStallCurrent = 2.5;
  public static final double P = 0;
  public static final double I = 0;
  public static final double D = 0;
}

public static final class WristConstants{
  public static final double homingStallCurrent = 2;
  public static final double P = 1;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0;
}

public static final class ElbowConstants{
  public static final double homingStallCurrent = 2.5;
  public static final double P = 0;
  public static final double I = 0;
  public static final double D = 0;
  public static final double G = 0;
}

public static final class ManipulatorConstants{
  public static final double coralStallCurrent = 7;
}


public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
}
  }


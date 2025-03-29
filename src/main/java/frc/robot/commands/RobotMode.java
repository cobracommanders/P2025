package frc.robot.commands;

public class RobotMode {
    public GameMode currentGameMode = GameMode.CORAL;

    public enum GameMode {CORAL, ALGAE}

    public void setCurrentGameMode(GameMode gameMode) {currentGameMode = gameMode;}
    public boolean inAlgaeMode() {return currentGameMode == GameMode.ALGAE;}
    public boolean inCoralMode() {return currentGameMode == GameMode.CORAL;}

    private static RobotMode instance;

  public static RobotMode getInstance() {
      if (instance == null) instance = new RobotMode(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}


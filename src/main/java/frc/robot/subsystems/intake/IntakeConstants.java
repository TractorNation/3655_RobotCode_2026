package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int CONVEYOR_ID = 21;
  public static final int KICKER_ID = 22;
  public static final int FRONT_MOTOR_ID = 23;
  public static final int TOP_MOTOR_ID = 24;
  public static final int BACK_MOTOR_ID = 25;
  public static final int SLIDER_ID = 26;

  public static final double SLIDER_RATIO = 25;
  public static final double SLIDER_KP = 0.25;
  public static final double SLIDER_KI = 0.0;
  public static final double SLIDER_KD = 0.0;

  public static final double MAX_SNOW_WALL_DISTANCE = 11;

  // todo: get real positions
  public static class SliderPositions {
    public static final double IN = 0.0;
    public static final double TRANSITION = 1;
    public static final double OUT = 2;
    public static final double BUMP_SAFE = 1.5;
  }

  public enum IntakeMode {
    INTAKE, OUTPUT, SNOWBLOWER
  }
  public enum IntakeState {
    TUCKED, BUMP_SAFE, TRANSITION, OUT
  }
}

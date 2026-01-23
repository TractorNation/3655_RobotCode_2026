package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
  public record TurretState(Rotation2d position, double shooterVelocityRotPerSec) {
  }

  public static final int TOP_RING_MOTOR_ID = 10;
  public static final int BOTTOM_RING_MOTOR_ID = 11;

  public static final double PLANET_GEAR_TO_TURRET_RATIO = 1.0;
  public static final double PLANET_GEAR_TO_SHOOTER_RATIO = 1.0;

  public static final double MOTOR_TO_RING_GEAR_RATIO = 1.0;

  public static final double TURRET_MAX_VELOCITY_ROT_PER_SEC = 2;
  public static final double TURRET_MAX_ACCELERATION_ROT_PER_SEC2 = 4;

  public static final double  TURRET_KP = 1.0;
  public static final double  TURRET_KI = 0.0;
  public static final double  TURRET_KD = 0.0;
  public static final double  TURRET_KV = 0.1;

  

}

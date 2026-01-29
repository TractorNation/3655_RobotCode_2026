package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
  public record TurretState(Rotation2d position, double shooterVelocityRotPerSec) {
  }

  public static final int TOP_RING_MOTOR_ID = 30;
  public static final int BOTTOM_RING_MOTOR_ID = 31;
  public static final int CANCODER_ID = 32;

  public static final double PLANET_GEAR_TO_TURRET_RATIO = 1.0;
  public static final double PLANET_GEAR_TO_SHOOTER_RATIO = 1.0;

  

  public static final double MOTOR_TO_RING_GEAR_RATIO = 28.8461538462;
  public static final double TURRET_TO_CANCODER_RATIO = 2.0208;

  public static final Rotation2d CANCODER_OFFSET = Rotation2d.fromRotations(0.0);

  public static final double TURRET_MAX_VELOCITY_ROT_PER_SEC = 2;
  public static final double TURRET_MAX_ACCELERATION_ROT_PER_SEC2 = 0.5;

  public static final double TURRET_KP = 1;
  public static final double TURRET_KI = 0.0;
  public static final double TURRET_KD = 0.0;
  public static final double TURRET_KV = 0.1;

}

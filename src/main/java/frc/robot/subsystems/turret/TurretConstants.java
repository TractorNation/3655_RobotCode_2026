package frc.robot.subsystems.turret;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

  public static final int TOP_RING_MOTOR_ID = 30;
  public static final int BOTTOM_RING_MOTOR_ID = 31;
  public static final int CANCODER_ID = 32;

  public static final double PLANET_GEAR_TO_TURRET_RATIO = 1.0;
  public static final double PLANET_GEAR_TO_SHOOTER_RATIO = 1.5 / 1;
  public static final double RING_GEAR_TO_PLANET_GEAR_RATIO = (double) 88 / 16;

  public static final double MOTOR_TO_RING_GEAR_RATIO = 6.2857;
  public static final double TURRET_TO_CANCODER_RATIO = 6.7692;

  public static final Rotation2d CANCODER_OFFSET = Rotation2d.fromRotations(0.0);

  public static final double TURRET_MAX_VELOCITY_ROT_PER_SEC = 4.0;
  public static final double TURRET_MAX_ACCELERATION_ROT_PER_SEC2 = 4;

  public static final double MOTOR_VELOCITY_KP = 1.5;
  public static final double MOTOR_VELOCITY_KI = 0.0;
  public static final double MOTOR_VELOCITY_KD = 0.00;
  public static final double MOTOR_VELOCITY_KS = 0.0;
  public static final double MOTOR_VELOCITY_KV = 0.0;

  public static final double POSITION_KP = 15;
  public static final double POSITION_KI = 0.0;
  public static final double POSITION_KD = 0.00;

  //TODO: set coordinates for blue and red hub
  public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.702, 3.987); 
  public static final Translation2d RED_HUB_POSITION = new Translation2d(11.926, 3.987);


  public static class TurretState {
    public double positionDegrees;
    public double shooterSpeedRotPerSec;

    public TurretState(double positionDegrees, double shooterSpeedRotPerSec) {
      this.positionDegrees = positionDegrees;
      this.shooterSpeedRotPerSec = shooterSpeedRotPerSec;
    }

    public double getPosition(){
      return positionDegrees;
    }

    public void setPosition(double newPosition){
      positionDegrees = newPosition;
    }

    public double getShooterSpeed(){
      return shooterSpeedRotPerSec;
    }

    public void setShooterSpeed(double newSpeed) {
      shooterSpeedRotPerSec = newSpeed;
    }
  }
}

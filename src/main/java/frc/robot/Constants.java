package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.PhysicsUtil;

public final class Constants {

  public static final class DeviceID {
    public static final String CANIVORE_NAME = "canivore";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);
    public static final int PIGEON_ID = 20;

    public static final class Turret {
      public static final int TOP_RING_MOTOR_ID = 30;
      public static final int BOTTOM_RING_MOTOR_ID = 31;
      public static final int CANCODER_ID = 32;
    }

    public static final class Intake {
      public static final int CONVEYOR_ID = 21;
      public static final int KICKER_ID = 22;
      public static final int FRONT_MOTOR_ID = 23;
      public static final int TOP_MOTOR_ID = 24;
      public static final int BACK_MOTOR_ID = 25;
      public static final int SLIDER_ID = 26;
    }

    public static final class Drive {
      public static final int FL_DRIVE_ID = 1;
      public static final int FL_TURN_ID = 2;
      public static final int FL_ENCODER_ID = 3;
      public static final int FR_DRIVE_ID = 4;
      public static final int FR_TURN_ID = 5;
      public static final int FR_ENCODER_ID = 6;
      public static final int BL_DRIVE_ID = 7;
      public static final int BL_TURN_ID = 8;
      public static final int BL_ENCODER_ID = 9;
      public static final int BR_DRIVE_ID = 10;
      public static final int BR_TURN_ID = 11;
      public static final int BR_ENCODER_ID = 12;
    }
  }

  public static final class PID {
    public static final class Turret {
      public static final double MOTOR_VELOCITY_KP = 2.5;
      public static final double MOTOR_VELOCITY_KI = 0.0;
      public static final double MOTOR_VELOCITY_KD = 0.00;
      public static final double MOTOR_VELOCITY_KS = 0.0;
      public static final double MOTOR_VELOCITY_KV = 0.0;
      public static final double TURRET_MAX_VELOCITY_ROT_PER_SEC = 4.0;
      public static final double TURRET_MAX_ACCELERATION_ROT_PER_SEC2 = 4;
      public static final double POSITION_KP = 15;
      public static final double POSITION_KI = 0.0;
      public static final double POSITION_KD = 0.00;
    }

    public static final class Intake {
      public static final double SLIDER_KP = 0.25;
      public static final double SLIDER_KI = 0.0;
      public static final double SLIDER_KD = 0.0;
    }

    public static final class Drive {
      public static final double KP_TURN = 100;
      public static final double KP_DRIVE = 0.3;
      public static final double KV_DRIVE = 0.71;
      public static final double KS_DRIVE = 0;
    }
  }

  public static final class OffsetAndRatio {
    public static final class Turret {
      public static final double PLANET_GEAR_TO_TURRET_RATIO = 1.0;
      public static final double PLANET_GEAR_TO_SHOOTER_RATIO = 1.5 / 1;
      public static final double RING_GEAR_TO_PLANET_GEAR_RATIO = (double) 88 / 16;
      public static final double MOTOR_TO_RING_GEAR_RATIO = 6.2857;
      public static final double TURRET_TO_CANCODER_RATIO = 6.7692;
    }

    public static final class Intake {
      public static final double SLIDER_RATIO = 25;
    }

    public static final class Drive {
      private static final Rotation2d PROTOBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.432129);
      private static final Rotation2d COMPBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.322510);
      private static final Rotation2d PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.111084);
      private static final Rotation2d COMPBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.431885);
      private static final Rotation2d PROTOBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.062500);
      private static final Rotation2d COMPBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.380127);
      private static final Rotation2d PROTOBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.485596);
      private static final Rotation2d COMPBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.278320);
      private static final double PROTOBOT_DRIVE_GEAR_RATIO = 5.27;
      private static final double COMPBOT_DRIVE_GEAR_RATIO = 5.27;
      private static final double PROTOBOT_TURN_GEAR_RATIO = 26.09;
      private static final double COMPBOT_TURN_GEAR_RATIO = 26.09;
      public static double DRIVE_GEAR_RATIO = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_DRIVE_GEAR_RATIO
          : PROTOBOT_DRIVE_GEAR_RATIO;
      public static double TURN_GEAR_RATIO = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_TURN_GEAR_RATIO
          : PROTOBOT_TURN_GEAR_RATIO;
      public static Rotation2d FRONT_LEFT_ENCODER_OFFSET = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_FRONT_LEFT_ENCODER_OFFSET
          : PROTOBOT_FRONT_LEFT_ENCODER_OFFSET;
      public static Rotation2d FRONT_RIGHT_ENCODER_OFFSET = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_FRONT_RIGHT_ENCODER_OFFSET
          : PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET;
      public static Rotation2d BACK_LEFT_ENCODER_OFFSET = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_BACK_LEFT_ENCODER_OFFSET
          : PROTOBOT_BACK_LEFT_ENCODER_OFFSET;
      public static Rotation2d BACK_RIGHT_ENCODER_OFFSET = (RobotConfig.currentRobot == RobotType.COMPBOT)
          ? COMPBOT_BACK_RIGHT_ENCODER_OFFSET
          : PROTOBOT_BACK_RIGHT_ENCODER_OFFSET;
    }
  }

  public static final class RobotConfig {
    public static final Driver currentDriver = Driver.MAIN;
    public static final RobotType currentRobot = RobotType.COMPBOT;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double BUMPER_WIDTH_X = Units.inchesToMeters(33.5);
    public static final double BUMPER_WIDTH_Y = Units.inchesToMeters(33.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);

    public static final double ODOMETRY_FREQUENCY = 250.0;

    public static final double MAX_LINEAR_SPEED = 5.768;
    public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS);

    public static final double BATTERY_MASS_KG = Units.lbsToKilograms(14);
    public static final double BUMPER_MASS_KG = Units.lbsToKilograms(16);
    private static final double COMPBOT_CHASSIS_MASS_KG = Units.lbsToKilograms(108.1);
    private static final double COMPBOT_MASS_KG = COMPBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
    private static final double COMPBOT_MOI = PhysicsUtil.estimateRobotMOI(
        COMPBOT_MASS_KG,
        BUMPER_WIDTH_X,
        BUMPER_WIDTH_Y);
    private static final double PROTOBOT_CHASSIS_MASS_KG = 24.05;
    private static final double PROTOBOT_MASS_KG = PROTOBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
    private static final double PROTOBOT_MOI = PhysicsUtil.estimateRobotMOI(
        PROTOBOT_MASS_KG,
        BUMPER_WIDTH_X,
        BUMPER_WIDTH_Y);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.984);
    public static final double WHEEL_COF = 1.5;
    public static final int DRIVE_CURRENT_LIMIT = 89;
    public static final int TURN_CURRENT_LIMIT = 15;
    public static double ROBOT_MASS_KG = (currentRobot == RobotType.COMPBOT)
        ? COMPBOT_MASS_KG
        : PROTOBOT_MASS_KG;
    public static double ROBOT_MOI = (currentRobot == RobotType.COMPBOT)
        ? COMPBOT_MOI
        : PROTOBOT_MOI;
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        // FL FR BL BR
        new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  }

  public static final class Vision {
    public static final double SINGLE_TAG_MAXIMUM = 3.5;
    public static final double MULTI_TAG_MAXIMUM = 7.5;

    public static final double MAX_AMBIGUITY = 0.25;

    public static final double LINEAR_STD_DEV_FACTOR = 0.8;
    public static final double ANGULAR_STD_DEV_FACTOR = 2;

    public static final double MEGATAG2_LINEAR_FACTOR = 0.25;
    public static final double MEGATAG2_ANGULAR_FACTOR = 2;

    public static final Translation3d LEFT_ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(-0.117, 0.2814, 0.2);
    public static final Rotation3d LEFT_ROBOT_TO_CAMERA_ROTATION = new Rotation3d(180, 28, -15);
    public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(LEFT_ROBOT_TO_CAMERA_TRANSLATION,
        LEFT_ROBOT_TO_CAMERA_ROTATION);

    public static final Translation3d RIGHT_ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(0.177, 0.214, 0.2);
    public static final Rotation3d RIGHT_ROBOT_TO_CAMERA_ROTATION = new Rotation3d(180, 28, 15);
    public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(RIGHT_ROBOT_TO_CAMERA_TRANSLATION,
        RIGHT_ROBOT_TO_CAMERA_ROTATION);
  }

  public static final class Field {
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.702, 3.987);
    public static final Translation2d RED_HUB_POSITION = new Translation2d(11.926, 3.987);
    public static final Rectangle2d BLUE_BUMP_ZONE = new Rectangle2d(new Translation2d(0, 0),
        new Translation2d(0, 0));
    public static final Rectangle2d RED_BUMP_ZONE = new Rectangle2d(new Translation2d(0, 0),
        new Translation2d(0, 0));
    public static final Rectangle2d RED_SCORING_ZONE = new Rectangle2d(
        new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(0)),
        new Translation2d(Units.inchesToMeters(651.22), Units.inchesToMeters(317.69)));
    public static final Rectangle2d BLUE_SCORING_ZONE = new Rectangle2d(new Translation2d(0, 0),
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69)));
    public static final double MAX_INTAKE_WALL_DISTANCE = 11;

  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum Driver {
    MAIN,
    PROGRAMMING,
    MACBOOK,
  }

  public static enum RobotType {
    COMPBOT,
    PROTOBOT
  }

  /**
   * A record to store position data for an observation.
   * 
   * @param tx The angle from the target on the x axis.
   * @param ty The angle from the target on the y axis.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
  }

  /**
   * A Record to store data about the robots pose calculated from vision
   * 
   * @param timestamp          The time in seconds when this observation was
   *                           recorded
   * @param pose               A pose 3d representing the robot's position on the
   *                           field
   * @param ambiguity          A value between 0 and 1 representing how confident
   *                           the vision system is in the pose where 0 is very
   *                           confident and 1 is not confident
   * @param tagCount           The number of tags that were used to calculate this
   *                           pose
   * @param averageTagDistance The average distance between tags used to calculate
   *                           this pose
   * 
   * @param type               The way the pose was calculated, used to calculate
   *                           standard deviations
   */
  public static record PoseObservation(
      double timestamp,
      Pose2d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      ObservationType type) {
  }

  public enum ObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTON
  }

  public static class TurretState {
    public double positionDegrees;
    public double shooterSpeedRotPerSec;

    public TurretState(double positionDegrees, double shooterSpeedRotPerSec) {
      this.positionDegrees = positionDegrees;
      this.shooterSpeedRotPerSec = shooterSpeedRotPerSec;
    }

    public double getPosition() {
      return positionDegrees;
    }

    public void setPosition(double newPosition) {
      positionDegrees = newPosition;
    }

    public double getShooterSpeed() {
      return shooterSpeedRotPerSec;
    }

    public void setShooterSpeed(double newSpeed) {
      shooterSpeedRotPerSec = newSpeed;
    }
  }

  // TODO: get real positions
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

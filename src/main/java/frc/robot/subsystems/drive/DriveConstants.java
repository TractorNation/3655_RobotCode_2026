package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PhysicsUtil;

/** Static values for the drive subsystem */
public class DriveConstants {

  public static final int FL_DRIVE_ID = 1;
  public static final int FL_TURN_ID = 2;
  public static final int FL_ENCODER_ID = 3;
  public static final Rotation2d PROTOBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.432129);
  public static final Rotation2d COMPBOT_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.431396);

  public static final int FR_DRIVE_ID = 4;
  public static final int FR_TURN_ID = 5;
  public static final int FR_ENCODER_ID = 6;
  public static final Rotation2d PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.111084);
  public static final Rotation2d COMPBOT_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.395996);

  public static final int BL_DRIVE_ID = 7;
  public static final int BL_TURN_ID = 8;
  public static final int BL_ENCODER_ID = 9;
  public static final Rotation2d PROTOBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.062500);
  public static final Rotation2d COMPBOT_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromRotations(0.062744);

  public static final int BR_DRIVE_ID = 10;
  public static final int BR_TURN_ID = 11;
  public static final int BR_ENCODER_ID = 12;
  public static final Rotation2d PROTOBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(-0.485596);
  public static final Rotation2d COMPBOT_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromRotations(0.483887);

  public static final int PIGEON_ID = 20;

  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static final double MAX_LINEAR_SPEED = 5.768;
  public static final double MAX_ANGULAR_SPEED = (MAX_LINEAR_SPEED / Constants.DRIVE_BASE_RADIUS);

  public static final double PROTOBOT_DRIVE_GEAR_RATIO = 5.27;

  public static final double COMPBOT_DRIVE_GEAR_RATIO = 5.27;

  public static final double TURN_GEAR_RATIO = 26.09;

  public static final double BATTERY_MASS_KG = Units.lbsToKilograms(14);
  public static final double BUMPER_MASS_KG = Units.lbsToKilograms(16);

  public static final double COMPBOT_CHASSIS_MASS_KG = Units.lbsToKilograms(108.1);
  public static final double COMPBOT_MASS_KG = COMPBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
  public static final double COMPBOT_MOI = PhysicsUtil.estimateRobotMOI(
      COMPBOT_MASS_KG,
      Constants.BUMPER_WIDTH_X,
      Constants.BUMPER_WIDTH_Y);

  public static final double PROTOBOT_CHASSIS_MASS_KG = 24.05;
  public static final double PROTOBOT_MASS_KG = PROTOBOT_CHASSIS_MASS_KG + BUMPER_MASS_KG + BATTERY_MASS_KG;
  public static final double PROTOBOT_MOI = PhysicsUtil.estimateRobotMOI(
      PROTOBOT_MASS_KG,
      Constants.BUMPER_WIDTH_X,
      Constants.BUMPER_WIDTH_Y);

  public static final double WHEEL_RADIUS = Units.inchesToMeters(1.984);
  public static final double WHEEL_COF = 1.5;

  public static final double KP_TURN = 100;
  public static final double KP_DRIVE = 0.3;
  public static final double KV_DRIVE = 0.71;
  public static final double KS_DRIVE = 0;

  public static final int DRIVE_CURRENT_LIMIT = 89;
  public static final int TURN_CURRENT_LIMIT = 15;

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      // FL FR BL BR
      new Translation2d(Constants.TRACK_WIDTH_X / 2.0, Constants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(Constants.TRACK_WIDTH_X / 2.0, -Constants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-Constants.TRACK_WIDTH_X / 2.0, Constants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-Constants.TRACK_WIDTH_X / 2.0, -Constants.TRACK_WIDTH_Y / 2.0)
  };

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

  public static double DRIVE_GEAR_RATIO = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_DRIVE_GEAR_RATIO
      : PROTOBOT_DRIVE_GEAR_RATIO;

  public static Rotation2d FRONT_LEFT_ENCODER_OFFSET = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_FRONT_LEFT_ENCODER_OFFSET
      : PROTOBOT_FRONT_LEFT_ENCODER_OFFSET;

  public static Rotation2d FRONT_RIGHT_ENCODER_OFFSET = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_FRONT_RIGHT_ENCODER_OFFSET
      : PROTOBOT_FRONT_RIGHT_ENCODER_OFFSET;

  public static Rotation2d BACK_LEFT_ENCODER_OFFSET = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_BACK_LEFT_ENCODER_OFFSET
      : PROTOBOT_BACK_LEFT_ENCODER_OFFSET;

  public static Rotation2d BACK_RIGHT_ENCODER_OFFSET = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_BACK_RIGHT_ENCODER_OFFSET
      : PROTOBOT_BACK_RIGHT_ENCODER_OFFSET;

  public static double ROBOT_MASS_KG = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_MASS_KG
      : PROTOBOT_MASS_KG;

  public static double ROBOT_MOI = (Constants.currentRobot == Constants.RobotType.COMPBOT)
      ? COMPBOT_MOI
      : PROTOBOT_MOI;

}
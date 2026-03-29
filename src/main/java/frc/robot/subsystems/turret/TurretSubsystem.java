package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretState;
import frc.robot.Constants;
import frc.robot.RobotState;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;

  private TurretState target;

  private ProfiledPIDController controller;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private double setpoint;

  @SuppressWarnings("unused")
  private Rectangle2d scoringZone;
  private Translation2d targetPosition;

  public static boolean shooterToggled = true;
  private double frameCount = 0;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();

    constraints = new TrapezoidProfile.Constraints(
        Constants.PID.Turret.TURRET_MAX_VELOCITY_ROT_PER_SEC,
        Constants.PID.Turret.TURRET_MAX_ACCELERATION_ROT_PER_SEC2);

    controller = new ProfiledPIDController(Constants.PID.Turret.POSITION_KP, Constants.PID.Turret.POSITION_KI,
        Constants.PID.Turret.POSITION_KD, constraints);

    target = new TurretState(0, 0);

    setTarget(180, 0);

    resetTarget();
  }

  @Override
  public void periodic() {
    if (frameCount < 100) {
      frameCount++;
      Logger.recordOutput("Turret/FrameCount", frameCount);
    } else {

      io.updateInputs(inputs);

      goalState = new TrapezoidProfile.State(Units.degreesToRotations(target.getPosition()), 0);

      setpoint = controller.calculate(inputs.turretPosition.getRotations(), goalState);

      double desiredTurretVelocity = setpoint
          * Constants.OffsetAndRatio.Turret.PLANET_GEAR_TO_TURRET_RATIO;

      double desiredShooterVelocity = target.getShooterSpeed()
          / Constants.OffsetAndRatio.Turret.RING_GEAR_TO_PLANET_GEAR_RATIO
          / Constants.OffsetAndRatio.Turret.PLANET_GEAR_TO_SHOOTER_RATIO;

      double topMotorTargetVelocity = desiredTurretVelocity + desiredShooterVelocity;
      double bottomMotorTargetVelocity = desiredTurretVelocity - desiredShooterVelocity;

      io.setTopRingMotorVelocity(topMotorTargetVelocity);
      io.setBottomRingMotorVelocity(bottomMotorTargetVelocity);

      Logger.recordOutput("Turret/CurrentPosition", inputs.turretPosition.getDegrees());
      Logger.recordOutput("Turret/targetPosition", target.getPosition());
      Logger.recordOutput("Turret/Shooter/CurrentVelocity", inputs.shooterVelocity);
      Logger.recordOutput("Turret/Shooter/TargetVelocity", target.getShooterSpeed());
      Logger.recordOutput("Turret/TopRingGear/Velocity", inputs.topRingMotorVelocity);
      Logger.recordOutput("Turret/TopRingGear/Target", topMotorTargetVelocity);
      Logger.recordOutput("Turret/BottomRingGear/Velocity", inputs.bottomRingMotorVelocity);
      Logger.recordOutput("Turret/BottomRingGear/Target", bottomMotorTargetVelocity);
      Logger.recordOutput("Turret/TopRingCurrent", inputs.topRingMotorCurrent);
      Logger.recordOutput("Turret/BottomRingCurrent", inputs.bottomRingMotorCurrent);
    }
  }

  public double wrapTarget(double targetPositionDegrees) {
    double currentPosition = inputs.turretPosition.getDegrees();
    double difference = targetPositionDegrees - currentPosition;

    while (difference > 180)
      difference -= 360;
    while (difference < -180)
      difference += 360;

    return currentPosition + difference;
  }

  public void setTarget(double targetPositionDegrees, double shooterVelocityRotPerSec) {
    target.setPosition(wrapTarget(targetPositionDegrees));
    target.setShooterSpeed(shooterVelocityRotPerSec);
  }

  public void targetHub() {
    double targetAngle;
    Pose2d futurePose = RobotState.getInstance().getFuturePose();
    Pose2d currentPose = RobotState.getInstance().getPose();
    Translation2d robotToTurret = new Translation2d(
        Constants.RobotConfig.ROBOT_TO_TURRET,
        Rotation2d.fromDegrees(currentPose.getRotation().getDegrees() + 135));
    Translation2d translation = futurePose.getTranslation().plus(robotToTurret);
    Translation2d robotToTarget = targetPosition.minus(translation);

    double shooterSpeed;

    if (shooterToggled) {
      shooterSpeed = Math.min((9.55 * robotToTarget.getNorm()) + 25.2, 85);
    } else {
      shooterSpeed = 0;
    }

    // Targets to the center of the hub, then adds an offset to account for the
    // ball's spin from the kicker
    targetAngle = (robotToTarget.getAngle().getDegrees() - currentPose.getRotation().getDegrees())
        + ((robotToTurret.getAngle().getDegrees() -
            180) / 90);

    Logger.recordOutput("Turret/DistanceToTarget", robotToTarget.getNorm());
    Logger.recordOutput("Turret/ShooterSpeedYesReal", shooterSpeed);
    setTarget(-targetAngle, shooterSpeed);
  }

  public void updateTarget(double value) {
    setTarget(target.positionDegrees + (value * 5), target.shooterSpeedRotPerSec);
  }

  public void incrementShooterSpeed(double increment) {
    setTarget(target.positionDegrees, increment);
  }

  public void stopMotors() {
    setTarget(target.positionDegrees, 0);
  }

  public void runShooter(double shooterSpeedRotPerSec) {
    setTarget(target.positionDegrees, shooterSpeedRotPerSec);
  }

  public void toggleShooter(boolean on) {
    shooterToggled = on;
  }

  public void changeTarget(Translation2d newTarget) {
    switch (DriverStation.getAlliance().get()) {
      case Blue:
        targetPosition = newTarget;
        break;
      case Red:
        targetPosition = new Translation2d(newTarget.getX() + 12.5, newTarget.getY());
        break;
      default:
        targetPosition = newTarget;
        break;
    }
  }

  public void resetTarget() {
    switch (DriverStation.getAlliance().get()) {
      case Red:
        targetPosition = Constants.Field.RED_HUB_POSITION;
        scoringZone = Constants.Field.RED_SCORING_ZONE;
        break;
      case Blue:
      default:
        targetPosition = Constants.Field.BLUE_HUB_POSITION;
        scoringZone = Constants.Field.BLUE_SCORING_ZONE;
    }
  }

  //Henry this method disappeared
  public TurretState getTarget() {
    return target;
  }
}
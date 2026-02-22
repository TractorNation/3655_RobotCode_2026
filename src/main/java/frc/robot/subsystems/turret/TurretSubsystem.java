package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants.TurretState;
import frc.robot.RobotState;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;

  private TurretState target;

  private ProfiledPIDController controller;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private double setpoint;
  private Rectangle2d scoringZone;
  private Translation2d hubPosition;
  private double shooterMultipler = 10;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();

    constraints = new TrapezoidProfile.Constraints(
        TurretConstants.TURRET_MAX_VELOCITY_ROT_PER_SEC, TurretConstants.TURRET_MAX_ACCELERATION_ROT_PER_SEC2);

    controller = new ProfiledPIDController(TurretConstants.POSITION_KP, TurretConstants.POSITION_KI,
        TurretConstants.POSITION_KD, constraints);

    target = new TurretState(0, 0);

    setTarget(0, 0);

    switch (DriverStation.getAlliance().get()) {
      case Red:
        hubPosition = TurretConstants.RED_HUB_POSITION;
        scoringZone = new Rectangle2d(new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(0)),
            new Translation2d(Units.inchesToMeters(651.22), Units.inchesToMeters(317.69)));
        break;
      case Blue:
      default:
        hubPosition = TurretConstants.BLUE_HUB_POSITION;
        scoringZone = new Rectangle2d(new Translation2d(0, 0),
            new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69)));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    goalState = new TrapezoidProfile.State(Units.degreesToRotations(target.getPosition()), 0);

    setpoint = controller.calculate(inputs.turretPosition.getRotations(), goalState);

    double desiredTurretVelocity = setpoint
        * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO;

    double desiredShooterVelocity = target.getShooterSpeed() / TurretConstants.RING_GEAR_TO_PLANET_GEAR_RATIO
        / TurretConstants.PLANET_GEAR_TO_SHOOTER_RATIO;

    double topMotorTargetVelocity = desiredTurretVelocity + desiredShooterVelocity;
    double bottomMotorTargetVelocity = desiredTurretVelocity - desiredShooterVelocity;

    io.setTopRingMotorVelocity(topMotorTargetVelocity);
    io.setBottomRingMotorVelocity(bottomMotorTargetVelocity);

    Logger.recordOutput("Turret/CurrentPosition", inputs.turretPosition.getDegrees());
    Logger.recordOutput("Turret/targetPosition", target.getPosition());
    Logger.recordOutput("Shooter/CurrentVelocity", inputs.shooterVelocity);
    Logger.recordOutput("Shooter/TargetVelocity", target.getShooterSpeed());
    Logger.recordOutput("Turret/TopRingGear/Velocity", inputs.topRingMotorVelocity);
    Logger.recordOutput("Turret/TopRingGear/Target", topMotorTargetVelocity);
    Logger.recordOutput("Turret/BottomRingGear/Velocity", inputs.bottomRingMotorVelocity);
    Logger.recordOutput("Turret/BottomRingGear/Target", bottomMotorTargetVelocity);
  }

  public double wrapTarget(double targetPositionDegrees) {
    double currentPosition = io.getTurretPosition();
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

  public void targetHub(double shooterSpeedRequest) {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Translation2d translation = currentPose.getTranslation();
    double targetAngle;
    double shooterSpeed = scoringZone.contains(translation) ? shooterSpeedRequest : 0;

    Translation2d robotToHub = hubPosition.minus(translation);

    targetAngle = robotToHub.getAngle().getDegrees() - currentPose.getRotation().getDegrees();

    setTarget(-targetAngle, 50);
  }

  public void updateTarget(double value) {
    setTarget(target.positionDegrees + (value * 5), target.shooterSpeedRotPerSec);
  }

  public void stopMotors() {
    setTarget(target.positionDegrees, 0);
  }

  public void runShooter(double shooterSpeedRotPerSec) {
    setTarget(target.positionDegrees, shooterSpeedRotPerSec);
  }
}
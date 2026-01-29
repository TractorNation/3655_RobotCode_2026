package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants.TurretState;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;

  private TurretState target;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State setpoint;

  private PIDController positionOffsetController;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged(); 

    positionOffsetController = new PIDController(12, 0, 0.05);

    this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        TurretConstants.TURRET_MAX_VELOCITY_ROT_PER_SEC, TurretConstants.TURRET_MAX_ACCELERATION_ROT_PER_SEC2));
    setTarget(0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    TrapezoidProfile.State currentState = new TrapezoidProfile.State(inputs.turretPosition.getRotations(),
        inputs.turretVelocity);

    setpoint = profile.calculate(0.02, currentState, goalState);

    double positionCorrection = positionOffsetController.calculate(
        inputs.turretPosition.getRotations(),
        goalState.position);

    // double positionError = goalState.position -
    // inputs.turretPosition.getRotations();
    // double positionCorrectionKp = 20;
    // double positionCorrection = positionError * positionCorrectionKp;

    double combinedTurretVelocity = setpoint.velocity + positionCorrection;

    double desiredTurretVelocity = combinedTurretVelocity
        * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO;
    double desiredShooterVelocity = target.shooterVelocityRotPerSec() * TurretConstants.PLANET_GEAR_TO_SHOOTER_RATIO;

    double topMotorTargetVelocity = desiredTurretVelocity + (desiredShooterVelocity / 2);
    double bottomMotorTargetVelocity = desiredTurretVelocity - (desiredShooterVelocity / 2);

    io.setTopRingMotorVelocity(topMotorTargetVelocity);
    io.setBottomRingMotorVelocity(bottomMotorTargetVelocity);

    Logger.recordOutput("Turret/CurrentPosition", inputs.turretPosition.getDegrees());
    Logger.recordOutput("Turret/targetPosition", target.position().getDegrees());
    Logger.recordOutput("Shooter/CurrentVelocity",
        inputs.shooterVelocity * TurretConstants.PLANET_GEAR_TO_SHOOTER_RATIO);
    Logger.recordOutput("Shooter/TargetVelocity", target.shooterVelocityRotPerSec());
  }

  public void setTarget(double targetPositionDegrees, double shooterVelocityRotPerSec) {
    double currentPosition = io.getTurretPosition();
    double difference = targetPositionDegrees - currentPosition;

    while (difference > 180)
      difference -= 360;
    while (difference < -180)
      difference += 360;

    double targetRotation = currentPosition + difference;

    target = new TurretState(Rotation2d.fromDegrees(targetRotation), shooterVelocityRotPerSec);
    goalState = new TrapezoidProfile.State(
        Units.degreesToRotations(targetRotation),
        0);
  }
}

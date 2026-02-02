package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants.TurretState;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;

  private TurretState target;

  private ProfiledPIDController controller;
  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private double setpoint;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();

    constraints = new TrapezoidProfile.Constraints(
        TurretConstants.TURRET_MAX_VELOCITY_ROT_PER_SEC, TurretConstants.TURRET_MAX_ACCELERATION_ROT_PER_SEC2);

    controller = new ProfiledPIDController(TurretConstants.POSITION_KP, TurretConstants.POSITION_KI,
        TurretConstants.POSITION_KD, constraints);

    target = new TurretState(0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    goalState = new TrapezoidProfile.State(Units.degreesToRotations(wrapTarget(target.getPosition())), 0);

    setpoint = controller.calculate(inputs.turretPosition.getRotations(), goalState);

    double desiredTurretVelocity = setpoint
        * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO;

    double desiredShooterVelocity = target.getShooterSpeed() * TurretConstants.RING_GEAR_TO_PLANET_GEAR_RATIO
        * TurretConstants.PLANET_GEAR_TO_SHOOTER_RATIO;

    double topMotorTargetVelocity = desiredTurretVelocity + desiredShooterVelocity;
    double bottomMotorTargetVelocity = desiredTurretVelocity - desiredShooterVelocity;

    io.setTopRingMotorVelocity(topMotorTargetVelocity);
    io.setBottomRingMotorVelocity(bottomMotorTargetVelocity);

    Logger.recordOutput("Turret/CurrentPosition", inputs.turretPosition.getDegrees());
    Logger.recordOutput("Turret/targetPosition", target.getPosition());
    Logger.recordOutput("Shooter/CurrentVelocity",
        inputs.shooterVelocity);
    Logger.recordOutput("Shooter/TargetVelocity", target.getShooterSpeed());
    Logger.recordOutput("Turret/TopRingGearVelocity", inputs.topRingMotorVelocity);  
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
    target.setPosition(targetPositionDegrees);
    target.setShooterSpeed(shooterVelocityRotPerSec);
  }
}
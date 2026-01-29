package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

  @AutoLog
  public class TurretIOInputs {
    public Rotation2d turretPosition = new Rotation2d();
    public double turretVelocity = 0.0;
    public double shooterVelocity = 0.0;
    public double topRingMotorPosition = 0.0;
    public double topRingMotorVelocity = 0.0;
    public double topRingMotorTemperature = 0.0;
    public double bottomRingMotorTemperature = 0.0;
    public double bottomRingMotorPosition = 0.0;
    public double bottomRingMotorVelocity = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {
  }

  public default void setTopRingMotorVelocity(double velocity) {
  }

  public default void setBottomRingMotorVelocity(double velocity) {
  }

  public default void stopShooter() {
  }

  public default double getTurretPosition() {
    return 0.0;
  }
}

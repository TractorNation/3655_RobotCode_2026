package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs {

        public double lowerRingVelocity = 0.0;

        public double upperRingVelocity = 0.0;
        public double upperRingTemp = 0.0;

        public Rotation2d turretPosition = new Rotation2d();

        public double theoreticalFlywheelSpeed = 0.0;

    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public default void setTurretState(double flywheelSpeed, double targetPosition) {}
}

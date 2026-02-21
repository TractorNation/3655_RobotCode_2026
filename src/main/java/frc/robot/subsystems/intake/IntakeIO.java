package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double frontMotorCurrent = 0.0;
    public double topMotorCurrent = 0.0;
    public double bottomMotorCurrent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void runIntakeMotors(double frontMotorSpeed, double topMotorSpeed, double backMotorSpeed) {
  }

  public default void stopMotors() {
  }

  public default void runIndexerMotors() {

  }

  public default void runConveyor(double speed) {}
}

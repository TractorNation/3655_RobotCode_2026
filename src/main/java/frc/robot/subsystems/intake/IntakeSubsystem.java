package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.FieldUtil;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private final double maxDistanceFromWall = 11;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runMotors(double frontMotorSpeed, double topMotorSpeed, double backMotorSpeed) {
    io.runIntakeMotors(frontMotorSpeed, topMotorSpeed, backMotorSpeed);
  }

  public void runIndexerMotors() {
    io.runIndexerMotors();
  }

  public void runConveyor(double speed) {
    io.runConveyor(speed);
  }

  public void stopMotors() {
    io.stopMotors();
  }

  public void runAgitator(double speed) {
    io.runAgitator(speed);
  }

  public void runSnowblower() {
    double distance = FieldUtil.getDistanceToWall(RobotState.getInstance().getPose());

    runMotors((0.5 / maxDistanceFromWall) * distance, (-1 / maxDistanceFromWall) * distance,
        (1 / maxDistanceFromWall) * distance); // + - +
  }
}

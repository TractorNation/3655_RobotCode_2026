package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Intake/SliderPosition", inputs.sliderPosition);
    Logger.recordOutput("Intake/IntakePosition", inputs.intakePosition);
    Logger.recordOutput("Intake/TopMotorCurrent", inputs.topMotorCurrent);
    Logger.recordOutput("Intake/BottomMotorCurrent", inputs.bottomMotorCurrent);
    Logger.recordOutput("Intake/FrontMotorCurrent", inputs.frontMotorCurrent);
    Logger.recordOutput("Intake/ConveyorCurrent", inputs.conveyorMotorCurrent);
    Logger.recordOutput("Intake/KickerCurrent", inputs.kickerMotorCurrent);
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

  public void runKicker(double speed) {
    io.runKicker(speed);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

}

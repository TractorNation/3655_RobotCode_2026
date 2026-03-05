package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private IntakeState state = IntakeState.TUCKED;

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

  public void runKicker(double speed) {
    io.runKicker(speed);
  }

  public void reverseIndexerMotors() {
    io.runKicker(0.7);
    io.runConveyor(-0.7);
  }

  public void setState(IntakeState state) {
    this.state = state;
    switch (state) {
      case TUCKED:
        io.setSliderPosition(IntakeConstants.SliderPositions.IN);
        break;
      case TRANSITION:
        // Set actual intake to its up position
        io.setSliderPosition(IntakeConstants.SliderPositions.TRANSITION);
        break;
      case OUT:
        io.setSliderPosition(IntakeConstants.SliderPositions.OUT);
        // set actual intake to its down position
        break;
      case BUMP_SAFE:
        io.setSliderPosition(IntakeConstants.SliderPositions.BUMP_SAFE);
        // maybe just rotate actual intake up a bit, have to figure that out later
      default:
        break;
    }
  }

  public IntakeState getState() {
    return state;
  }

}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private IntakeState state = IntakeState.TUCKED;
  private IntakeState preBumpSafeState = state;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (state != IntakeState.BUMP_SAFE) {
      preBumpSafeState = state;
    }

    switch (state) {
      case TUCKED:
        Logger.recordOutput("Intake/Target", Constants.SliderPositions.IN);
        io.setSliderPosition(Constants.SliderPositions.IN);
        // io.setIntakePosition(Constants.IntakePositions.UP);
        break;

      case TRANSITION:
        Logger.recordOutput("Intake/Target", Constants.SliderPositions.TRANSITION);
        // io.setIntakePosition(Constants.IntakePositions.UP);
        io.setSliderPosition(Constants.SliderPositions.TRANSITION);
        break;
      case OUT:
        Logger.recordOutput("Intake/Target", Constants.SliderPositions.OUT);
        io.setSliderPosition(Constants.SliderPositions.OUT);
        // io.setIntakePosition(Constants.IntakePositions.DOWN);
        break;
      case BUMP_SAFE:
        Logger.recordOutput("Intake/Target", Constants.SliderPositions.BUMP_SAFE);
        io.setSliderPosition(Constants.SliderPositions.BUMP_SAFE);
        // maybe just rotate actual intake up a bit, have to figure that out later
      default:
        break;
    }

    Logger.recordOutput("Intake/SliderPosition", inputs.sliderPosition);
    Logger.recordOutput("Intake/IntakePosition", inputs.intakePosition);
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
  }

  public void setArmPosition(double position) {
    io.setIntakePosition(position);
  }

  @AutoLogOutput(key = "Intake/CurrentPosition")
  public IntakeState getState() {
    return state;
  }

  public void staySafeFromBump() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Translation2d translation = currentPose.getTranslation();

    if (Constants.Field.RED_BUMP_ZONE.contains(translation) || Constants.Field.BLUE_BUMP_ZONE.contains(translation)) {
      setState(IntakeState.BUMP_SAFE);
    } else {
      setState(preBumpSafeState);
    }
  }
}

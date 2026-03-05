package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private IntakeState state = IntakeState.TUCKED;
  private IntakeState preBumpSafeState = state;
  private Rectangle2d redBumps = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
  private Rectangle2d blueBumps = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));

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
    if (state != IntakeState.BUMP_SAFE) {
      preBumpSafeState = state;
    }
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

  public void staySafeFromBump() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Translation2d translation = currentPose.getTranslation();
    
    if(redBumps.contains(translation) || blueBumps.contains(translation)) {
      setState(IntakeState.BUMP_SAFE);
    } else {
      setState(preBumpSafeState);
    }
  }
}

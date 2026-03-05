package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeMode;
import frc.robot.Constants.IntakeState;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {

  public static Command runIntakeMode(IntakeSubsystem intake, IntakeMode mode) {

    switch (mode) {
      case INTAKE:
        return Commands.runOnce(() -> intake.runMotors(0.4, 0.4, 0.4), intake); // + + +
      case OUTPUT:
        return runOutput(intake);
      case SNOWBLOWER:
        return runSnowblower(intake);
      default:
        return Commands.none();
    }
  }

  public static Command setIntakePosition(IntakeSubsystem intake, IntakeState pos) {
    IntakeState currentState = intake.getState();
    if (currentState == pos || (currentState == IntakeState.TUCKED && pos == IntakeState.BUMP_SAFE)) {
      return Commands.none();
    }

    switch (pos) {
      case TUCKED:
        return Commands.sequence(
            Commands.runOnce(() -> intake.setState(IntakeState.TRANSITION), intake),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> intake.setState(IntakeState.TUCKED), intake));
      case OUT:
        return Commands.sequence(
            Commands.runOnce(() -> intake.setState(IntakeState.TRANSITION), intake),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> intake.setState(IntakeState.OUT), intake));
      case BUMP_SAFE:
        return Commands.runOnce(() -> intake.setState(IntakeState.BUMP_SAFE), intake);
      default:
        return Commands.none();

    }
  }

  public static Command runIndexer(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.runIndexerMotors(), intake);
  }

  public static Command reverseIndexer(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.reverseIndexerMotors(), intake);
  }

  public static Command stopIntake(IntakeSubsystem intake) {
    return Commands.runOnce(() -> {
      intake.stopMotors();
    }, intake);
  }

  public static Command runSnowblower(IntakeSubsystem intake) {
    final double distance = RobotState.getInstance().getDistanceToWall();

    return Commands.run(() -> intake.runMotors((0.5 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
        (-1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
        (1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance), intake);
  }

  public static Command runOutput(IntakeSubsystem intake) {
    final double distance = RobotState.getInstance().getDistanceToWall();
    return Commands.run(() -> {
      intake.runMotors((1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
          (-0.75 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
          (-1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance); // + - -
      intake.runConveyor(-0.6);
    }, intake);
  }

  public static Command staySafeFromBump(IntakeSubsystem intake) {
    return Commands.run(() -> intake.staySafeFromBump(), intake);
  }
}

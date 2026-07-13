package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeMode;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {

  public static Command runIntakeMode(IntakeSubsystem intake, IntakeMode mode) {

    switch (mode) {
      case INTAKE:
        return Commands.runOnce(() -> intake.runMotors(0.4, 0.4), intake); // + + +
      case OUTPUT:
        return Commands.runOnce(() -> {
          intake.runMotors(0.75, -0.4);
          intake.runConveyor(0.6);
        }, intake);
      case SNOWBLOWER:
        return runSnowblower(intake);
      default:
        return Commands.none();
    }
  }

  public static Command runIndexer(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.runIndexerMotors());
  }

  public static Command runIndexerInAuto(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.runIndexerMotors(), intake);
  }

  public static Command runConveyorBackwards(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.runConveyorBackwards(), intake);
  }

  public static Command stopIntake(IntakeSubsystem intake) {
    return Commands.runOnce(() -> {
      intake.stopMotors();
    }, intake);
  }

  public static Command runSnowblower(IntakeSubsystem intake) {
    final double distance = RobotState.getInstance().getDistanceToWall();

    return Commands.run(() -> intake.runMotors((0.5 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
        (-1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance), intake);
  }

  public static Command runOutput(IntakeSubsystem intake) {
    final double distance = RobotState.getInstance().getDistanceToWall();
    return Commands.run(() -> {
      intake.runMotors((1 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance,
          (-0.75 / Constants.Field.MAX_INTAKE_WALL_DISTANCE) * distance); // + - -
      intake.runConveyor(0.6);
    }, intake);
  }

  public static Command setPosition(IntakeSubsystem intake, double position) {
    return Commands.runOnce(() -> intake.setPosition(position), intake);
  }

  public static Command runKicker(IntakeSubsystem intake, double speed) {
    return Commands.runOnce(() -> intake.runKicker(speed), intake);
  }

  public static Command runConveyor(IntakeSubsystem intake, double speed) {
    return Commands.runOnce(() -> intake.runConveyor(speed), intake);
  }

}

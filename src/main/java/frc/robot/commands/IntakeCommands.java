package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;

public class IntakeCommands {

  public static Command runIntake(IntakeSubsystem intake, IntakeMode mode) {
    final double intakeSpeed = 0.5;

    return Commands.runOnce(() -> {
      switch (mode) {
        case INTAKE:
          intake.runMotors(intakeSpeed, intakeSpeed, intakeSpeed); // + + + 
          break;
        case OUTPUT:
          intake.runMotors(intakeSpeed, -intakeSpeed, intakeSpeed); // + - -
          break;
        case LOBSHOT: 
          intake.runMotors(1, -0.5, -0.8); // + - -
          break;
        case LONGSHOT:
          intake.runMotors(intakeSpeed, intakeSpeed, -intakeSpeed); // + + -
          break;
        case SNOWBLOWER:
          intake.runMotors(1, -1, intakeSpeed); // + - +
          break;
        default:
          break;
      }
    }, intake);
  }

  public static Command stopIntake(IntakeSubsystem intake) {
    return Commands.runOnce(() -> {
      intake.stopMotors();
    }, intake);
  }
}

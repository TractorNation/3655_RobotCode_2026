package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TakeBallAndShootPrettyPlease {

    public static Command run(TurretSubsystem turret, IntakeSubsystem intake, double shooterVelocityRotPerSec) {
        return Commands.run(() -> {
            intake.runIndexerMotors();
            turret.runShooter(shooterVelocityRotPerSec);
        }, turret, intake);
    }

    public static Command stop(TurretSubsystem turret, IntakeSubsystem intake) {
        return Commands.run(() -> {
            intake.stopMotors();
            turret.stopMotors();
        }, turret, intake);
    }
}

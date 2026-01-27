package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretCommands {

  public static Command updateState(TurretSubsystem turret, double turretPositionDegrees, double shooterVelocityRotPerSec) {
    return Commands.runOnce(() -> turret.setTarget(turretPositionDegrees, shooterVelocityRotPerSec), turret);
  }
}

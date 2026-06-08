package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretCommands {

  public static Command updateState(TurretSubsystem turret, double turretPositionDegrees,
      double shooterVelocityRotPerSec) {
    return Commands.runOnce(() -> turret.setTarget(turretPositionDegrees, shooterVelocityRotPerSec), turret);
  }

  public static Command trackTarget(TurretSubsystem turret) {
    return Commands.run(() -> turret.targetHub(), turret);
  }

  public static Command trackTag(TurretSubsystem turret) {
    return Commands.run(() -> turret.targetTag(), turret);
  }

  public static Command toggleShooter(TurretSubsystem turret, boolean on) {
    return Commands.runOnce(() -> turret.toggleShooter(on), turret);
  }

  public static Command changeTarget(TurretSubsystem turret, Translation2d newTarget) {
    return Commands.runOnce(() -> turret.changeTarget(newTarget), turret);
  }

  public static Command targetHub(TurretSubsystem turret) {
    return Commands.runOnce(() -> turret.resetTarget(), turret);
  }
}

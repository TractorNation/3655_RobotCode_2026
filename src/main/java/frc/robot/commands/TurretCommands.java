package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretConstants.TurretState;

public class TurretCommands {

  public static Command updateState(TurretSubsystem turret, TurretState state) {
    return Commands.runOnce(() -> turret.setTarget(state), turret);
  }
}

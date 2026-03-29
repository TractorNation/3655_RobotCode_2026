// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateTurretState extends Command {
  TurretSubsystem turret;
  double targetRPS;
  double increment;
  double targetPosition;

  /** Creates a new UpdateTurretState. */
  public UpdateTurretState(TurretSubsystem turret, double positionDegrees, double shooterVelocityRotPerSec) {
    this.turret = turret;
    targetRPS = shooterVelocityRotPerSec;
    targetPosition = positionDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setTarget(targetPosition, increment);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.incrementShooterSpeed(1);
    increment++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.getTarget().shooterSpeedRotPerSec >= targetRPS;
  }
}

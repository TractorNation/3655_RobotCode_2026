package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOSim implements TurretIO {
  private double topRingVelocity = 0;
  private double bottomRingVelocity = 0;
  private double topRingPosition = 0;
  private double bottomRingPosition = 0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    
    topRingPosition += topRingVelocity * 0.02;
    bottomRingPosition += bottomRingVelocity * 0.02;

    inputs.turretPosition = Rotation2d.fromRotations(
        ((topRingPosition + bottomRingPosition) / 2.0)
            * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO);

    inputs.turretVelocity = ((topRingVelocity + bottomRingVelocity) / 2.0)
        * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO;

    inputs.shooterVelocity = topRingVelocity - bottomRingVelocity;

    inputs.topRingMotorPosition = topRingPosition;
    inputs.topRingMotorVelocity = topRingVelocity;
    inputs.bottomRingMotorPosition = bottomRingPosition;
    inputs.bottomRingMotorVelocity = bottomRingVelocity;
  }

  @Override
  public void setTopRingMotorVelocity(double velocity) {
    topRingVelocity = velocity; 
  }

  @Override
  public void setBottomRingMotorVelocity(double velocity) {
    bottomRingVelocity = velocity;
  }
}

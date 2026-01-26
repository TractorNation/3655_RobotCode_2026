package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX topRingMotor;
  private final TalonFX bottomRingMotor;

  private final StatusSignal<Angle> topRingAngle;
  private final StatusSignal<Angle> bottomRingAngle;
  private final StatusSignal<AngularVelocity> topRingVelocity;
  private final StatusSignal<AngularVelocity> bottomRingVelocity;
  private final StatusSignal<Temperature> topRingTemperature;
  private final StatusSignal<Temperature> bottomRingTemperature;

  public TurretIOTalonFX() {
    topRingMotor = new TalonFX(TurretConstants.TOP_RING_MOTOR_ID);
    bottomRingMotor = new TalonFX(TurretConstants.BOTTOM_RING_MOTOR_ID);

    var config = new TalonFXConfiguration();

    config.Slot0.kP = TurretConstants.TURRET_KP;
    config.Slot0.kI = TurretConstants.TURRET_KI;
    config.Slot0.kD = TurretConstants.TURRET_KD;
    config.Slot0.kV = TurretConstants.TURRET_KV;
    config.Feedback.SensorToMechanismRatio = TurretConstants.MOTOR_TO_RING_GEAR_RATIO;

    topRingMotor.getConfigurator().apply(config);
    bottomRingMotor.getConfigurator().apply(config);

    topRingAngle = topRingMotor.getPosition();
    bottomRingAngle = bottomRingMotor.getPosition();
    topRingVelocity = topRingMotor.getVelocity();
    bottomRingVelocity = bottomRingMotor.getVelocity();
    topRingTemperature = topRingMotor.getDeviceTemp();
    bottomRingTemperature = bottomRingMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topRingAngle,
        topRingVelocity,
        topRingTemperature,
        bottomRingAngle,
        bottomRingVelocity,
        bottomRingTemperature);
    topRingMotor.optimizeBusUtilization();
    bottomRingMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topRingAngle,
        topRingVelocity,
        topRingTemperature,
        bottomRingAngle,
        bottomRingVelocity,
        bottomRingTemperature);

    inputs.topRingMotorPosition = topRingAngle.getValueAsDouble();
    inputs.topRingMotorVelocity = topRingVelocity.getValueAsDouble();
    inputs.topRingMotorTemperature = topRingTemperature.getValueAsDouble();
    inputs.bottomRingMotorPosition = bottomRingAngle.getValueAsDouble();
    inputs.bottomRingMotorVelocity = bottomRingVelocity.getValueAsDouble();
    inputs.bottomRingMotorTemperature = bottomRingTemperature.getValueAsDouble();

    inputs.turretPosition = Rotation2d.fromRotations(
        ((topRingMotor.getPosition().getValueAsDouble() + bottomRingMotor.getPosition().getValueAsDouble()) / 2.0)
            * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO);

    inputs.turretVelocity = ((topRingMotor.getVelocity().getValueAsDouble()
        + bottomRingMotor.getVelocity().getValueAsDouble()) / 2.0)
        * TurretConstants.PLANET_GEAR_TO_TURRET_RATIO;

    inputs.shooterVelocity = topRingMotor.getVelocity().getValueAsDouble()
        - bottomRingMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setTopRingMotorVelocity(double velocity) {
    topRingMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setBottomRingMotorVelocity(double velocity) {
    bottomRingMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void stopShooter() {
    topRingMotor.setControl(new VelocityVoltage(0.0));
    bottomRingMotor.setControl(new VelocityVoltage(0.0));
  }
}

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX topRingMotor;
  private final TalonFX bottomRingMotor;
  private final CANcoder encoder;

  private final StatusSignal<AngularVelocity> topRingVelocity;
  private final StatusSignal<AngularVelocity> bottomRingVelocity;
  private final StatusSignal<Angle> canCoderPosition;

  public TurretIOTalonFX() {
    topRingMotor = new TalonFX(TurretConstants.TOP_RING_MOTOR_ID);
    bottomRingMotor = new TalonFX(TurretConstants.BOTTOM_RING_MOTOR_ID);
    encoder = new CANcoder(TurretConstants.CANCODER_ID);

    var config = new TalonFXConfiguration();

    config.Slot0.kP = TurretConstants.MOTOR_VELOCITY_KP;
    config.Slot0.kI = TurretConstants.MOTOR_VELOCITY_KI;
    config.Slot0.kD = TurretConstants.MOTOR_VELOCITY_KD;
    config.Slot0.kV = TurretConstants.MOTOR_VELOCITY_KV;
    config.Slot0.kS = TurretConstants.MOTOR_VELOCITY_KS;
    config.Feedback.SensorToMechanismRatio = TurretConstants.MOTOR_TO_RING_GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET.getRotations();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    topRingMotor.getConfigurator().apply(config);
    bottomRingMotor.getConfigurator().apply(config);

    topRingVelocity = topRingMotor.getVelocity();
    bottomRingVelocity = bottomRingMotor.getVelocity();
    canCoderPosition = encoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topRingVelocity,
        bottomRingVelocity,
        canCoderPosition);
    topRingMotor.optimizeBusUtilization();
    bottomRingMotor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topRingVelocity,
        bottomRingVelocity,
        canCoderPosition);

    double topRingVelocityRPS = topRingVelocity.getValueAsDouble();
    double bottomRingVelocityRPS = bottomRingVelocity.getValueAsDouble();

    inputs.topRingMotorVelocity = topRingVelocity.getValueAsDouble();
    inputs.bottomRingMotorVelocity = bottomRingVelocity.getValueAsDouble();

    inputs.turretPosition = Rotation2d.fromRotations(canCoderPosition.getValueAsDouble()
        / TurretConstants.TURRET_TO_CANCODER_RATIO);

    inputs.shooterVelocity = (((topRingVelocityRPS
        - bottomRingVelocityRPS) * TurretConstants.RING_GEAR_TO_PLANET_GEAR_RATIO)
        * TurretConstants.PLANET_GEAR_TO_SHOOTER_RATIO);
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

  @Override
  public double getTurretPosition() {
    return Units.rotationsToDegrees(
        canCoderPosition.getValueAsDouble()
            / TurretConstants.TURRET_TO_CANCODER_RATIO);
  }
}

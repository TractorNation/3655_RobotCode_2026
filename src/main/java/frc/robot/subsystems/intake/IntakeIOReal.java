package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex frontMotor = new SparkFlex(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex backMotor = new SparkFlex(IntakeConstants.BACK_MOTOR_ID, MotorType.kBrushless);

  private final TalonFX conveyorMotor = new TalonFX(IntakeConstants.CONVEYOR_ID);
  private final TalonFX kickerMotor = new TalonFX(IntakeConstants.KICKER_ID);
  private final TalonFX sliderMotor = new TalonFX(IntakeConstants.SLIDER_ID);

  SparkMaxConfig frontConfig;
  SparkMaxConfig topConfig;
  SparkMaxConfig backConfig;

  TalonFXConfiguration sliderConfig;
  StatusSignal<Angle> sliderPosition;

  public IntakeIOReal() {
    frontConfig = new SparkMaxConfig();
    topConfig = new SparkMaxConfig();
    backConfig = new SparkMaxConfig();
    frontConfig.inverted(true);
    topConfig.inverted(true);
    backConfig.inverted(false);
    frontMotor.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backMotor.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sliderConfig = new TalonFXConfiguration();
    sliderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    sliderConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SLIDER_RATIO;
    sliderConfig.Slot0.kP = IntakeConstants.SLIDER_KP;
    sliderConfig.Slot0.kI = IntakeConstants.SLIDER_KI;
    sliderConfig.Slot0.kD = IntakeConstants.SLIDER_KD;
    sliderMotor.getConfigurator().apply(sliderConfig);

    sliderPosition = sliderMotor.getPosition();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sliderPosition.refresh();

    inputs.frontMotorCurrent = frontMotor.getOutputCurrent();
    inputs.topMotorCurrent = topMotor.getOutputCurrent();
    inputs.bottomMotorCurrent = backMotor.getOutputCurrent();
    inputs.sliderPosition = sliderPosition.getValueAsDouble();
  }

  @Override
  public void runIntakeMotors(double frontMotorSpeed, double topMotorSpeed, double backMotorSpeed) {
    frontMotor.set(frontMotorSpeed);
    topMotor.set(topMotorSpeed);
    backMotor.set(backMotorSpeed);
  }

  @Override
  public void stopMotors() {
    frontMotor.stopMotor();
    topMotor.stopMotor();
    backMotor.stopMotor();
    conveyorMotor.stopMotor();
    kickerMotor.stopMotor();
  }

  @Override
  public void runIndexerMotors() {
    runKicker(-0.75);
    runConveyor(0.4);
  }

  @Override
  public void runConveyor(double speed) {
    conveyorMotor.set(speed);
  }

  @Override
  public void runKicker(double speed) {
    kickerMotor.set(speed);
  }

  @Override
  public void setSliderPosition(double position) {
    sliderMotor.setControl(new PositionVoltage(position));
  }

}

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex frontMotor = new SparkFlex(Constants.DeviceID.Intake.FRONT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(Constants.DeviceID.Intake.TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex backMotor = new SparkFlex(Constants.DeviceID.Intake.BACK_MOTOR_ID, MotorType.kBrushless);

  private final TalonFX conveyorMotor = new TalonFX(Constants.DeviceID.Intake.CONVEYOR_ID);
  private final TalonFX kickerMotor = new TalonFX(Constants.DeviceID.Intake.KICKER_ID);
  private final TalonFX sliderMotor = new TalonFX(Constants.DeviceID.Intake.SLIDER_ID);
  private final TalonFX rightArmMotor = new TalonFX(Constants.DeviceID.Intake.ARM_RIGHT);
  private final TalonFX leftArmMotor = new TalonFX(Constants.DeviceID.Intake.ARM_LEFT);

  SparkMaxConfig frontConfig;
  SparkMaxConfig topConfig;
  SparkMaxConfig backConfig;

  TalonFXConfiguration sliderConfig;
  TalonFXConfiguration armConfig;
  StatusSignal<Angle> sliderPosition;
  StatusSignal<Angle> intakePosition;

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
    sliderConfig.Feedback.SensorToMechanismRatio = Constants.OffsetAndRatio.Intake.SLIDER_RATIO;
    sliderConfig.Slot0.kP = Constants.PID.Intake.SLIDER_KP;
    sliderConfig.Slot0.kI = Constants.PID.Intake.SLIDER_KI;
    sliderConfig.Slot0.kD = Constants.PID.Intake.SLIDER_KD;
    sliderConfig.Slot0.kV = 0.025;
    sliderConfig.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    sliderConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    sliderConfig.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    sliderConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
    sliderConfig.MotionMagic.MotionMagicAcceleration = 160;
    sliderConfig.MotionMagic.MotionMagicJerk = 1600;
    sliderMotor.getConfigurator().apply(sliderConfig);

    armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio = Constants.OffsetAndRatio.Intake.ARM_RATIO;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.kP = Constants.PID.Intake.INTAKE_KP;
    armConfig.Slot0.kI = Constants.PID.Intake.INTAKE_KI;
    armConfig.Slot0.kD = Constants.PID.Intake.INTAKE_KD;

    leftArmMotor.getConfigurator().apply(armConfig);
    rightArmMotor.getConfigurator().apply(armConfig);

    leftArmMotor.setControl(new Follower(Constants.DeviceID.Intake.ARM_RIGHT, MotorAlignmentValue.Aligned));

    sliderPosition = sliderMotor.getPosition();
    intakePosition = rightArmMotor.getPosition();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(sliderPosition, intakePosition);

    inputs.frontMotorCurrent = frontMotor.getOutputCurrent();
    inputs.topMotorCurrent = topMotor.getOutputCurrent();
    inputs.bottomMotorCurrent = backMotor.getOutputCurrent();
    inputs.sliderPosition = sliderPosition.getValueAsDouble();
    inputs.intakePosition = intakePosition.getValueAsDouble();
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
    sliderMotor.setControl(new MotionMagicVoltage(0).withPosition(position));
  }

  @Override
  public void setIntakePosition(double position) {
    rightArmMotor.setControl(new PositionVoltage(position));
  }

}

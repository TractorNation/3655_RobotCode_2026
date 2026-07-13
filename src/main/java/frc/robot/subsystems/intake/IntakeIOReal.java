package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.RobotState;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex frontMotor = new SparkFlex(Constants.DeviceID.Intake.FRONT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(Constants.DeviceID.Intake.TOP_MOTOR_ID, MotorType.kBrushless);
  private final TalonFX armMotor = new TalonFX(Constants.DeviceID.Intake.ARM_ID);
  private final SparkFlex conveyorMotor = new SparkFlex(Constants.DeviceID.Intake.CONVEYOR_ID, MotorType.kBrushless);
  private final TalonFX kickerMotor = new TalonFX(Constants.DeviceID.Intake.KICKER_ID);

  SparkMaxConfig frontConfig;
  SparkMaxConfig topConfig;
  TalonFXConfiguration armConfig;

  StatusSignal<Current> kickerCurrent;
  StatusSignal<Angle> position;

  double positionRotations = 0.2;

  public IntakeIOReal() {
    frontConfig = new SparkMaxConfig();
    topConfig = new SparkMaxConfig();
    frontConfig.inverted(true);
    topConfig.inverted(true);
    frontMotor.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio = Constants.OffsetAndRatio.Intake.ARM_RATIO;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.Slot0.kP = Constants.PID.Intake.INTAKE_KP;
    armConfig.Slot0.kI = Constants.PID.Intake.INTAKE_KI;
    armConfig.Slot0.kD = Constants.PID.Intake.INTAKE_KD;
    armConfig.Slot0.kG = Constants.PID.Intake.INTAKE_KG;

    armMotor.getConfigurator().apply(armConfig);

    armMotor.setPosition(0.25);

    position = armMotor.getPosition();
    kickerCurrent = kickerMotor.getSupplyCurrent();

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, kickerCurrent);
    inputs.frontMotorCurrent = frontMotor.getOutputCurrent();
    inputs.topMotorCurrent = topMotor.getOutputCurrent();
    inputs.intakePosition = position.getValueAsDouble();
    inputs.kickerMotorCurrent = kickerCurrent.getValueAsDouble();

    //armMotor.setControl(new PositionVoltage(positionRotations));
  }

  @Override
  public void runIntakeMotors(double frontMotorSpeed, double topMotorSpeed) {
    frontMotor.set(frontMotorSpeed);
    topMotor.set(topMotorSpeed);
  }

  @Override
  public void stopMotors() {
    frontMotor.stopMotor();
    topMotor.stopMotor();
    conveyorMotor.stopMotor();
    kickerMotor.stopMotor();
  }

  @Override
  public void runIndexerMotors() {
    runKicker(-0.9);
    runConveyor(-0.75);
  }

  @Override
  public void runConveyorBackwards(){
    runConveyor(0.5);
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
  public void setPosition(double position) {
    positionRotations = position;
  }

  @Override
  public void automateKickerSpeed(){
    Translation2d robotToHub = RobotState.getInstance().getRobotToHub();

    // Max turret speed at 120rps
    // TODO: Write equation to change kicker speed based on distance to hub (shooter speed)
  }

}

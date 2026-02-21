package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex frontMotor = new SparkFlex(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex backMotor = new SparkFlex(IntakeConstants.BACK_MOTOR_ID, MotorType.kBrushless);

  private final TalonFX conveyorMotor = new TalonFX(IntakeConstants.CONVEYOR_ID);
  private final TalonFX kickerMotor = new TalonFX(IntakeConstants.KICKER_ID);

  SparkMaxConfig frontConfig;
  SparkMaxConfig topConfig; 
  SparkMaxConfig backConfig;

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
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.frontMotorCurrent = frontMotor.getOutputCurrent();
    inputs.topMotorCurrent = topMotor.getOutputCurrent();
    inputs.bottomMotorCurrent = backMotor.getOutputCurrent();
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
    conveyorMotor.set(0.8);
    kickerMotor.set(-0.6);
  }

  @Override
  public void runConveyor(double speed) {
    conveyorMotor.set(speed);
  }

}

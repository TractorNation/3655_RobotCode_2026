package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Current;

public class IntakeIOReal implements IntakeIO {
  private final SparkFlex frontMotor = new SparkFlex(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex topMotor = new SparkFlex(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final TalonFX backMotor = new TalonFX(IntakeConstants.BACK_MOTOR_ID);

  private StatusSignal<Current> backMotorCurrent = backMotor.getSupplyCurrent();

  public IntakeIOReal() {
    var backConfig = new TalonFXConfiguration();

    backConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    backMotor.getConfigurator().apply(backConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    backMotorCurrent.refresh();

    inputs.frontMotorCurrent = frontMotor.getOutputCurrent();
    inputs.topMotorCurrent = topMotor.getOutputCurrent();
    inputs.bottomMotorCurrent = backMotorCurrent.getValueAsDouble();
  }

  @Override
  public void runMotors(double frontMotorSpeed, double topMotorSpeed, double backMotorSpeed) {
    frontMotor.set(frontMotorSpeed);
    topMotor.set(topMotorSpeed);
    backMotor.set(backMotorSpeed);
  }

  @Override
  public void stopMotors() {
    frontMotor.stopMotor();
    topMotor.stopMotor();
    backMotor.set(0.0);
  }

}

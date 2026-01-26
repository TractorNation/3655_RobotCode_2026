// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class TurretIOTalonFX implements TurretIO {

    private final TalonFX upperRingTalon;
    private final TalonFX lowerRingTalon;
    private final CANcoder turretCancoder;

    private final StatusSignal<AngularVelocity> upperRingVelocity;
    private final StatusSignal<AngularVelocity> lowerRingVelocity;

    private final TalonFXConfiguration upperRingConfig;
    private final TalonFXConfiguration lowerRingConfig;
    private final CANcoderConfiguration cancoderConfig;


    public TurretIOTalonFX() {

        upperRingTalon = new TalonFX(TurretConstants.UPPER_RING_MOTOR_PORT, CANBus.roboRIO());
        lowerRingTalon = new TalonFX(TurretConstants.LOWER_RING_MOTOR_PORT, CANBus.roboRIO());
        turretCancoder = new CANcoder(TurretConstants.TURRET_CANCODER_PORT, CANBus.roboRIO());

        upperRingVelocity = upperRingTalon.getVelocity();
        lowerRingVelocity = lowerRingTalon.getVelocity();

        upperRingConfig = new TalonFXConfiguration();
        lowerRingConfig = new TalonFXConfiguration();
        cancoderConfig = new CANcoderConfiguration();

        upperRingConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        upperRingConfig.Feedback.SensorToMechanismRatio = TurretConstants.UPPER_RING_GEAR_RATIO;
        upperRingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        lowerRingConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        lowerRingConfig.Feedback.SensorToMechanismRatio = TurretConstants.LOWER_RING_GEAR_RATIO;
        lowerRingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cancoderConfig.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_GEAR_RATIO;
        //TODO: Check this sensor direction
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        BaseStatusSignal.refreshAll(upperRingVelocity, lowerRingVelocity);

        inputs.upperRingVelocity = upperRingVelocity.getValueAsDouble();
        inputs.lowerRingVelocity = lowerRingVelocity.getValueAsDouble();
    }

    @Override
    public void setTurretState(double flywheelSpeed, double targetPosition) {

        double deltaTheta = turretCancoder.getPosition().getValueAsDouble() - targetPosition;

        
    }


}

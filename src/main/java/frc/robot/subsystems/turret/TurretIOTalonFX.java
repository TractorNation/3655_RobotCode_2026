// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class TurretIOTalonFX {

    private final TalonFX upperRingTalon;
    private final TalonFX lowerRingTalon;
    private final CANcoder turretCancoder;

    private final StatusSignal<AngularVelocity> upperRingVelocity;
    private final StatusSignal<AngularVelocity> lowerRingVelocity;

    private final TalonFXConfiguration upperRingConfig;
    private final TalonFXConfiguration lowerRingConfig;


    public TurretIOTalonFX() {

        upperRingTalon = new TalonFX(TurretConstants.UPPER_RING_MOTOR_PORT, "rio");
        lowerRingTalon = new TalonFX(TurretConstants.LOWER_RING_MOTOR_PORT, "rio");
        turretCancoder = new CANcoder(TurretConstants.TURRET_CANCODER_PORT, "rio");

        upperRingVelocity = upperRingTalon.getVelocity();
        lowerRingVelocity = lowerRingTalon.getVelocity();

        upperRingConfig = new TalonFXConfiguration();
        lowerRingConfig = new TalonFXConfiguration();

    }


}

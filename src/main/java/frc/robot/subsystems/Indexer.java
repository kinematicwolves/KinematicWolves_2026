// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
    // Here, we declare all the motors, sensors, and other objects that this subsystem needs
    // Declaring all the motors for the Indexer
    private final TalonFX rollerMotor = new TalonFX(56); //TODO: Replace with actual CAN ID
    private final TalonFX feederMotor = new TalonFX(57); //TODO: Replace with actual CAN ID

    
    public Indexer() {
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure rollerMotor
        configureRollerMotor();

        // Configure feederMotor
        configureFeederMotor();
    }

    /* Private, internal functions */
    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Inversion
        // TODO: Determine which one of these is correct
        // I recommend setting this such that a positive number moves the game piece toward the launcher
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Neutral Mode
        // we typically set rollers to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current Limits
        // TODO: Adjust Current Limits as nessesssary
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        this.rollerMotor.getConfigurator().apply(config);
    }

    private void configureFeederMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Inversion
        // TODO: Determine which one of these is correct
        // I recommend setting this such that a positive number moves a game piece into the launcher
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Neutral Mode
        // we typically set rollers to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current Limits
        // TODO: Adjust Current Limits as nessesssary
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        this.feederMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /* Public functions for commands */
    /**
     * Stops the indexer and feeder motors.
     */
    public void stop() {
        this.rollerMotor.stopMotor();
    }

    /**
     * Sets the speed of the roller motor.
     * 0 is off, 1 is full speed one direction, -1 is full speed the other direction
     * @param speedFraction between -1 and 1
     */
    public void setRollerSpeed(double speedFraction) {
        this.rollerMotor.set(speedFraction);
    }

    /**
     * Sets the speed of the feeder motor.
     * 0 is off, 1 is full speed one direction, -1 is full speed the other direction
     * @param speedFraction between -1 and 1
     */
    public void setFeederSpeed(double speedFraction) {
        this.feederMotor.set(speedFraction);
    }
}

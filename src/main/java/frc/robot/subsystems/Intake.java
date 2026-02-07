// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    // Here, we declare all the motors, sensors, and other objects that this subsystem needs
    // Declaring all the motors for the intake
    private final TalonFX intakeMotor1 = new TalonFX(31); //TODO: Replace with actual CAN ID
    private final TalonFX intakeMotor2 = new TalonFX(32); //TODO: Replace with actual CAN ID
    
    public Intake() {
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure intakeMotor1
        configureIntakeMotor1();

        // Configure intakeMotor2
        configureIntakeMotorB();

    }

    /* Private, internal functions */
    private void configureIntakeMotor1() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID
        config.Slot0.kP = 0.3; // TODO: Replace with 1/(max motor range from smart dashboard)
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TODO: change to True when motor limits are known
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100.0; // rotations //TODO: Determine the range for the motor, then change this value
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TODO: change to True when motor limits are known
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // rotations //TODO: Determine the range for the motor, then change this value

        // Neutral mode
        // TODO: I've set this to coast for now, maybe change to break later
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // TODO: Enable which ever is correct for this motor
        // I suggest this be set such that when the intake moves outside the robot, that is positive
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // apply the config
        this.intakeMotor1.getConfigurator().apply(config);

        // reset the sensor
        this.intakeMotor1.setPosition(0.0);
    }

    private void configureIntakeMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Neutral mode
        // TODO: I've set this to coast for now, maybe change to break later. This should match intakeMotor1
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // TODO: Enable which ever is correct for this motor
        // I suggest this be set such that when the intake moves outside the robot, that is positive
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // apply the config
        this.intakeMotor2.getConfigurator().apply(config);

        // Tell this motor to follow whatever intakeMotor1 does
        this.intakeMotor2.setControl(new Follower(this.intakeMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // This is a good spot to put any state-monitoring, smart dashboard outputs, logging, etc
        SmartDashboard.putNumber("Intake Position", this.intakeMotor1.getPosition().getValueAsDouble()); // puts the intake position on the smart dashboard
    }

    /* Public functions for commands */
    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     */
    public void setPosition(double rotations) {
        this.intakeMotor1.setControl(new PositionVoltage(rotations));
    }
     
    /**
     * Returns if the intake is at its target position.
     * @return True if the intake is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.intakeMotor1.getClosedLoopError().getValue()) < 0.2; // TODO: Configure tolerance, move to constants file
    }
}

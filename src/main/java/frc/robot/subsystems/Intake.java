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
    private final TalonFX intakeMotorA = new TalonFX(50); //TODO: Replace with actual CAN ID
    private final TalonFX intakeMotorB = new TalonFX(51); //TODO: Replace with actual CAN ID
    private final TalonFX rollerMotor  = new TalonFX(52); //TODO: Replace with actual CAN ID
    
    public Intake() {
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure intakeMotorA
        configureIntakeMotorA();

        // Configure intakeMotorB
        configureIntakeMotorB();

        // Configure rollerMotor
        configureRollerMotor();
    }

    /* Private, internal functions */
    private void configureIntakeMotorA() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID
        config.Slot0.kP = 0.0; // TODO: Replace with 1/(max motor range from smart dashboard)
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
        this.intakeMotorA.getConfigurator().apply(config);

        // reset the sensor
        this.intakeMotorA.setPosition(0.0);
    }

    private void configureIntakeMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Neutral mode
        // TODO: I've set this to coast for now, maybe change to break later. This should match intakeMotorA
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // TODO: Enable which ever is correct for this motor
        // I suggest this be set such that when the intake moves outside the robot, that is positive
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // apply the config
        this.intakeMotorB.getConfigurator().apply(config);

        // Tell this motor to follow whatever intakeMotorA does
        this.intakeMotorB.setControl(new Follower(this.intakeMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Inversion
        // TODO: Determine which one of these is correct
        // I recommend setting this such that a positive number intakes a game piece
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Neutral Mode
        // we typically set rollers to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // This is a good spot to put any state-monitoring, smart dashboard outputs, logging, etc
        SmartDashboard.putNumber("Intake Position", this.intakeMotorA.getPosition().getValueAsDouble()); // puts the intake position on the smart dashboard
        SmartDashboard.putNumber("Intake Speed",    this.rollerMotor.get()); // puts the roller speed on the smart dashboard
    }

    /* Public functions for commands */
    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     */
    public void setPosition(double rotations) {
        this.intakeMotorA.setControl(new PositionVoltage(rotations));
    }
    
    /**
     * Returns if the intake is at its target position.
     * @return True if the intake is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.intakeMotorA.getClosedLoopError().getValue()) < 0.2; // TODO: Configure tolerance, move to constants file
    }

    /**
     * Stops the intake roller motor
     */
    public void stop() {
        this.rollerMotor.stopMotor();
    }
    
    /**
     * Sets the speed of the intake roller.
     * 0 is off, 1 is full speed one direction, -1 is full speed the other direction
     * @param speedFraction between -1 and 1
     */
    public void setSpeed(double speedFraction) {
        this.rollerMotor.set(speedFraction);
    }
}

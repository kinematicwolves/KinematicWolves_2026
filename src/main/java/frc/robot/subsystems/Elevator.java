// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorProfile;
import frc.robot.generated.TunerConstants;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */
    private TalonFX motorA = new TalonFX(ElevatorProfile.motorACanID, TunerConstants.kCANBus);
    private TalonFX motorB = new TalonFX(ElevatorProfile.motorBCanID, TunerConstants.kCANBus);

    private boolean breakOn = false;
 
    public Elevator() {
        configureMotorA();

        configureMotorB();

        configureMotionMagic();
    }

    private void configureMotorA() {
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/MotionMagic/src/main/java/frc/robot/Robot.java
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100.0; // rotations //TODO: Determine elevator range
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // rotations

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // I suggest positive values make elevator go up
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // apply the config
        this.motorA.getConfigurator().apply(config);

        // reset the sensor
        this.motorA.setPosition(0.0);
    }

    private void configureMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // I suggest positive values make elevator go up
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // apply the config
        this.motorB.getConfigurator().apply(config);

        // reset the sensor
        this.motorB.setPosition(0.0);

        // Tell this motor to follow whatever intakeMotorA does
        this.motorB.setControl(new Follower(this.motorA.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureMotionMagic() {
        /* Configure Motion Magic */
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(5) // rotations per second cruise
        .withMotionMagicAcceleration(10) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(100);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ElevatorPose", this.motorA.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("ElevatorAtPose", this.atSetpoint());
        SmartDashboard.putNumber("ElevatorSpeed", this.motorA.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ElevatorAPose", this.motorA.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ElevatorBPose", this.motorB.getPosition().getValueAsDouble());
    }

    /**
     * Sets the speed of the elevator as a percent of motor output.
     * @param percent the speed of the motors, -1 to 1.
     */
    public void setPercent(double percent) {
        this.motorA.set(percent);
    }

    /* Public functions for commands */
    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     * @param slot the pid slot to use. 0 for high pid, 1 for low kp
     */
    public void setPosition(double rotations, int slot) {
        // this.motorA.setControl(new PositionVoltage(rotations).withSlot(slot));
    }
     
    /**
     * Returns if the elevator is at its target position.
     * @return True if the elevator is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.motorA.getClosedLoopError().getValue()) < 0.1; //TODO: put into constants file
    }

    public void setBreakMode(boolean brakeOn) {
        this.breakOn = brakeOn;

        if (this.breakOn) {
            this.motorA.setNeutralMode(NeutralModeValue.Brake);
            this.motorB.setNeutralMode(NeutralModeValue.Brake);
        }
        else {
            this.motorA.setNeutralMode(NeutralModeValue.Coast);
            this.motorB.setNeutralMode(NeutralModeValue.Coast);

        }
    }

    public void togglBreakMode() {
        this.setBreakMode(!this.breakOn);
    }
}

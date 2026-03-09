// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private TalonFX motor1 = new TalonFX(ElevatorProfile.motor1ID, TunerConstants.kCANBus);
    private TalonFX motor2 = new TalonFX(ElevatorProfile.motor2ID, TunerConstants.kCANBus);

    public Elevator() {
        configureMotor1();

        configureMotor2();

        configureMotionMagic();
    }

    private void configureMotor1() {
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
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.0; // rotations //TODO: Determine elevator range
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // rotations

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Setting the motor direction
        // I suggest positive values make elevator go up
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // apply the config
        this.motor1.getConfigurator().apply(config);

        // reset the sensor
        this.motor1.setPosition(0.0);
    }

    private void configureMotor2() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Setting the motor direction
        // I suggest positive values make elevator go up
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // apply the config
        this.motor2.getConfigurator().apply(config);

        // Tell this motor to follow whatever intakeMotor1 does
        this.motor2.setControl(new Follower(this.motor1.getDeviceID(), MotorAlignmentValue.Opposed));
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
        SmartDashboard.putNumber("ElevatorPose", this.motor1.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("ElevatorAtPose", this.atSetpoint());
        SmartDashboard.putNumber("ElevatorSpeed", this.motor1.getVelocity().getValueAsDouble());
    }

    /**
     * Sets the speed of the elevator as a percent of motor output.
     * @param percent the speed of the motors, -1 to 1.
     */
    public void setPercent(double percent) {
        this.motor1.set(percent);
    }

    /* Public functions for commands */
    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     * @param slot the pid slot to use. 0 for high pid, 1 for low kp
     */
    public void setPosition(double rotations, int slot) {
        // this.motor1.setControl(new PositionVoltage(rotations).withSlot(slot));
    }
     
    /**
     * Returns if the elevator is at its target position.
     * @return True if the elevator is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.motor1.getClosedLoopError().getValue()) < 0.1; //TODO: put into constants file
    }

}

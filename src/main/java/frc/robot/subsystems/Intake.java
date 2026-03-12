// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeProfile;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    // Here, we declare all the motors, sensors, and other objects that this subsystem needs
    // Declaring all the motors for the intake
    private final TalonFX  intakeMotor1 = new TalonFX(IntakeProfile.intakeMotorACanID, TunerConstants.kCANBus);
    private final TalonFX  intakeMotor2 = new TalonFX(IntakeProfile.intakeMotorBCanID, TunerConstants.kCANBus);
    private final TalonSRX roller       = new TalonSRX(IntakeProfile.rollerMotorCanID);
    
    public Intake() {
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure intakeMotor1
        configureIntakeMotor1();

        // Configure intakeMotor2
        configureIntakeMotorB();

        configureRollorMotor();

    }

    /* Private, internal functions */
    private void configureIntakeMotor1() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID
        config.Slot0.kP = 0.3;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Slot1.kP = 0.6;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeProfile.deployedPose;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeProfile.retractedPose;

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Setting the motor direction
        // I suggest this be set such that when the intake moves outside the robot, that is positive
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // apply the config
        this.intakeMotor1.getConfigurator().apply(config);

        // reset the sensor
        this.intakeMotor1.setPosition(0.0);
    }

    private void configureIntakeMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Setting the motor direction
        // I suggest this be set such that when the intake moves outside the robot, that is positive
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // apply the config
        this.intakeMotor2.getConfigurator().apply(config);

        // Tell this motor to follow whatever intakeMotor1 does
        this.intakeMotor2.setControl(new Follower(this.intakeMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureRollorMotor() {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.continuousCurrentLimit = 20;
        roller.configAllSettings(config);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // This is a good spot to put any state-monitoring, smart dashboard outputs, logging, etc
        SmartDashboard.putNumber("Intake Pose", this.intakeMotor1.getPosition().getValueAsDouble()); // puts the intake position on the smart dashboard
        SmartDashboard.putBoolean("IntakeAtPose", this.atSetpoint()); // puts the intake position on the smart dashboard
    }

    /* Public functions for commands */
    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     */
    public void setPosition(double rotations, int slot) {
        this.intakeMotor1.setControl(new PositionVoltage(rotations).withSlot(slot));
    }
     
    /**
     * Returns if the intake is at its target position.
     * @return True if the intake is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.intakeMotor1.getClosedLoopError().getValue()) < IntakeProfile.poseTolerance;
    }

    /**
     * Sets the speed percent of the roller in open-loop-mode.
     * @param percet between -1 and 1
     */
    public void setRollerPercent(double percet) {
        this.roller.set(TalonSRXControlMode.PercentOutput, percet);
    }

    /**
     * Resests the intake zero as the current position of the intake
     */
    public void zeroIntake() {
        this.intakeMotor1.setPosition(0.0);
    }
}

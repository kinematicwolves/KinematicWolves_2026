// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    
    // --- Hardware ---
    private final TalonFX  intakeMotor1 = new TalonFX(IntakeProfile.intakeMotorACanID, TunerConstants.kCANBus);
    private final TalonFX  intakeMotor2 = new TalonFX(IntakeProfile.intakeMotorBCanID, TunerConstants.kCANBus);
    private final TalonSRX roller       = new TalonSRX(IntakeProfile.rollerMotorCanID);
    
    public Intake() {
        configurePivotMotors();
        configureRollerMotor();
    }

    private void configurePivotMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // PID Slot 0 (Gentle / Retract)
        config.Slot0.kP = 0.3;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.2;

        // PID Slot 1 (Aggressive / Deploy)
        config.Slot1.kP = 0.6;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        config.Slot1.kG = 0.2; 

        // Current Limits (Crucial to apply to BOTH motors)
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeProfile.deployedPose;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeProfile.retractedPose;

        // Neutral mode & Direction
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Apply config to Motor 1
        this.intakeMotor1.getConfigurator().apply(config);
        
        // Apply the EXACT SAME config to Motor 2 to ensure identical current limits
        this.intakeMotor2.getConfigurator().apply(config);

        // Set Motor 2 to strictly follow Motor 1
        this.intakeMotor2.setControl(new Follower(this.intakeMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Reset the sensor assuming the robot boots up completely retracted
        this.intakeMotor1.setPosition(0.0);
    }

    private void configureRollerMotor() {
        // Factory default wipes any lingering configurations from other projects
        this.roller.configFactoryDefault();
        
        // Set to coast so game pieces don't get jammed when the motor stops
        this.roller.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        
        // Protect the 775 motor from burning out
        this.roller.configContinuousCurrentLimit(25);
        this.roller.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pose", this.intakeMotor1.getPosition().getValueAsDouble()); 
        SmartDashboard.putBoolean("IntakeAtPose", this.atSetpoint()); 
        SmartDashboard.putNumber("Intake Roller Current", this.roller.getStatorCurrent());
    }

    /* Public functions for commands */

    /**
     * Sets the target position of the intake in motor rotations
     * @param rotations the position of the intake in motor rotations
     * @param slot the pid slot to use. 0 for gentle, 1 for aggressive
     */
    public void setPosition(double rotations, int slot) {
        this.intakeMotor1.setControl(new PositionVoltage(rotations).withSlot(slot));
    }
     
    /**
     * Returns if the intake is at its target position.
     * @return True if the intake is at its set position, otherwise false
     */
    public boolean atSetpoint() {
        return Math.abs(this.intakeMotor1.getClosedLoopError().getValueAsDouble()) < IntakeProfile.poseTolerance;
    }

    /**
     * Sets the speed percent of the roller in open-loop-mode.
     * @param percent between -1 and 1
     */
    public void setRollerPercent(double percent) {
        this.roller.set(TalonSRXControlMode.PercentOutput, percent);
    }

    /**
     * Resets the intake zero as the current position of the intake
     */
    public void zeroIntake() {
        this.intakeMotor1.setPosition(0.0);
    }
}
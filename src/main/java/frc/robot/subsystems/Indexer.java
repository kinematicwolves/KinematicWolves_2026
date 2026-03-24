// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerProfile;

public class Indexer extends SubsystemBase {

    // --- Hardware ---
    private final SparkMax kickerMotor = new SparkMax(IndexerProfile.kickerMotorCanID, SparkLowLevel.MotorType.kBrushless);
    private final TalonSRX roller      = new TalonSRX(IndexerProfile.rollerMotorCanID);

    public Indexer() {
        configureKickerMotor();
        configureRollerMotor();
    }

    private void configureKickerMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // Current Limits
        config.smartCurrentLimit(30);

        // Neutral Mode & Direction
        config.idleMode(IdleMode.kCoast); 
        config.inverted(true);

        // PID & Feedforward for Velocity Control
        config.closedLoop
            .p(0.0001, ClosedLoopSlot.kSlot0) // TODO: Tune this. P should be very small for velocity!
            .i(0.0, ClosedLoopSlot.kSlot0)
            .d(0.0, ClosedLoopSlot.kSlot0);
        
        // Apply the config ONCE, resetting old parameters and saving the new ones
        this.kickerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        // Factory default to clear any old settings
        this.roller.configFactoryDefault();
        
        // Coast mode prevents game pieces from getting stuck
        this.roller.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        
        // Protect the 775 motor!
        this.roller.configContinuousCurrentLimit(20);
        this.roller.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Velocity [RPM]", kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Roller Current", roller.getStatorCurrent());
    }

    /**
     * Sets the speed of the motor in closed loop mode
     * @param speed the speed for the motor, rpm
     */
    public void setKickerSpeed(double speed) {
        kickerMotor.getClosedLoopController().setSetpoint(speed, SparkMax.ControlType.kVelocity);
    }

    /**
     * Sets the speed of the motor in open loop mode
     * @param percent the speed for the motor, -1 to 1
     */
    public void setKickerPercent(double percent) {
        kickerMotor.set(percent);
    }  

    /**
     * Sets the speed of the roller in open-loop-mode
     * @param percent the speed for the motor, -1 to 1
     */
    public void setRollerPercent(double percent) {
        roller.set(TalonSRXControlMode.PercentOutput, percent);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerProfile;

public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
    // Step one, create all the objects we need
    private final SparkMax kickerMotor = new SparkMax(IndexerProfile.kickerMotorID, SparkLowLevel.MotorType.kBrushless);
    private final TalonSRX roller      = new TalonSRX(IndexerProfile.rollerMotorID);

    public Indexer() {
        configureKickerMotor();
    }

    private void configureKickerMotor() {
        // https://docs.revrobotics.com/brushless/spark-max/parameters
        // the spark max is configured differently than talon fx motors
        SparkMaxConfig config = new SparkMaxConfig();

        // first, we clear the current parameters
        this.kickerMotor.configure(config, ResetMode.kResetSafeParameters, null);

        // Current Limits
        config.smartCurrentLimit(20);

        //Neutral Mode
        config.idleMode(IdleMode.kCoast); 
        
        // Setting the motor direction
        config.inverted(true);

        // configuring the pid controller for the hood angle
        
        // finally, we apply our config to as persistent parameters
        this.kickerMotor.configure(config, null, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("KickerSpeed [RPM]", this.kickerMotor.getEncoder().getVelocity());
    }

    /**
     * Sets the roller speed percentage in open-loop mode
     * @param percent between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setKickerPercent(double percent) {
        this.kickerMotor.set(percent);
    }

    /**
     * Sets the roller speed percentage in open-loop mode
     * @param percent between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setRollerPercent(double percent) {
        this.roller.set(TalonSRXControlMode.PercentOutput, percent);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
    private final SparkMax kickerMotor = new SparkMax(IndexerProfile.kickerMotorCanID, SparkLowLevel.MotorType.kBrushless);
    private final TalonSRX roller      = new TalonSRX(IndexerProfile.rollerMotorCanID);

    private RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
    
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
        config.smartCurrentLimit(30);

        //Neutral Mode    
        config.idleMode(IdleMode.kCoast); 
        
        // Setting the motor direction
        config.inverted(true);

        //set the pid gains
        config.closedLoop
        .p(0.0001)
        .i(0.00001)
        .d(0.1);
        
        // finally, we apply our config to as persistent parameters
        this.kickerMotor.configure(config, null, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("KickerVelocity[RPM]", kickerEncoder.getVelocity());
    }

    /**
     * Sets the speeed of the motor in closed loop mode
     * @param speed the speed for the motor, rpm
     */
    public void setKickerSpeed(double speed) {
        kickerMotor.getClosedLoopController().setSetpoint(speed, SparkMax.ControlType.kVelocity);
    }

    /**
     * sets the speed of the motor in open loop mode
     * @param percent the speed for the motor, -1 to 1
     */
    public void setKickerPercent(double percent) {
        kickerMotor.set(percent);
    }  

    /**
     * Tets the speed of the roller in open-loop-mode
     * @param percent the speed for the motor, -1 to 1
     */
    public void setRollerPercent(double percent) {
        roller.set(TalonSRXControlMode.PercentOutput, percent);
    }
}

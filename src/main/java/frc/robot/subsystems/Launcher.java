// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Launcher extends SubsystemBase {
    /** Creates a new Launcher. */

    // Step one, create all the objects we need
    private final TalonFX launcherMotor1 = new TalonFX(41, TunerConstants.kCANBus); 
    private final TalonFX launcherMotor2 = new TalonFX(42, TunerConstants.kCANBus); 

    // hood motor and controller objects
    private final SparkMax hoodMotor = new SparkMax(44, SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController hoodPIDController = hoodMotor.getClosedLoopController();

    public Launcher() {
        // Step 2, apply whatever configs we need
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure LauncherMotorA
        configureLauncherMotor1();

        // Configure LauncherMotorB
        configureLauncherMotor2();

        configureHoodMotor();
    }

    /* Private, internal functions */
    private void configureLauncherMotor1() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID (Velocity)
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html#velocity-control
        // config.Slot0.kS = 0.0; // TODO: increase until the motors overcome friction in the launcher
        // config.Slot0.kV = 0.0; // TODO: set to 0.12
        // config.Slot0.kP = 0.0; // TODO: set to 1/(max speed)
        // config.Slot0.kI = 0.0;
        // config.Slot0.kD = 0.0;

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Neutral Mode
        // We typically set flywheels to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // I suggest this be set such that a positive number launcher the game piece
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying the config
        this.launcherMotor1.getConfigurator().apply(config);
    }

    private void configureLauncherMotor2() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Neutral Mode
        // We typically set flywheels to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // I suggest this be set such that a positive number launcher the game piece
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying the config
        this.launcherMotor2.getConfigurator().apply(config);

        // set motorB to follow motorA
        this.launcherMotor2.setControl(new Follower(this.launcherMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureHoodMotor() {
        // https://docs.revrobotics.com/brushless/spark-max/parameters
        // the spark max is configured differently than talon fx motors
        SparkMaxConfig config = new SparkMaxConfig();

        // first, we clear the current parameters
        this.hoodMotor.configure(config, ResetMode.kResetSafeParameters, null);

        // Current Limits
        config.smartCurrentLimit(20);

        //Neutral Mode
        // Set to coast fist so we can move the hood by hand.
        // After we determine the range, we can change it back to brake mode
        // we typically set hoods to break mode so key keep their position
        config.idleMode(IdleMode.kBrake); 
                
        // Setting the motor direction
        config.inverted(false);

        // setting the forward / reverse position limits so the hood doesn't rip itself apart... hopefully :)
        // config.softLimit.forwardSoftLimit(0); //TODO: Change limits to based on real hardware
        // config.softLimit.forwardSoftLimitEnabled(false); //TODO: Change to true to enable
        // config.softLimit.reverseSoftLimit(0); //TODO: Change limits to based on real hardware (this one will probably stay at 0)
        // config.softLimit.reverseSoftLimitEnabled(false); //TODO: Change to true to enable

        // configuring the pid controller for the hood angle
        config.closedLoop
        .p(0.1)
        .i(0.0)
        .d(0.0);

        // finally, we apply our config to as persistent parameters
        this.hoodMotor.configure(config, null, PersistMode.kPersistParameters);

        this.hoodMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("HoodPose", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("HoodAtSetPoint", hoodIsAtSetpoint());
        SmartDashboard.putNumber("HoodTarget", hoodPIDController.getSetpoint());
        SmartDashboard.putNumber("Launcher/Velocity RPS", launcherMotor1.getVelocity().getValueAsDouble());
    }

    /* Public functions for commands */
    /**
     * Sets the flywheel percentage in open-loop mode. Do not use for launching game pieces. This is to help with setup and testing.
     * @param speedFraction between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setFlywheelPercent(double speedFraction) {
        this.launcherMotor1.set(speedFraction);
    }

    /**
     * Sets the hood's target position
     * @param rotations the position in the hood in rotations
     */
    public void setHoodPosition(double rotations) {
        this.hoodPIDController.setReference(rotations, SparkMax.ControlType.kPosition);
    }

    /**
     * Returns if the hood is at its set point or not.
     * @return True if the hood is at its setpoint, false otherwise.
     */
    public boolean hoodIsAtSetpoint() {
        // Unlike the TalonFX, the spark max doesn't have a function call for how close it is.
        // Therefore, we will look at the the current position, and compare it to the stored setpoint
        return Math.abs(this.hoodMotor.getEncoder().getPosition() - this.hoodPIDController.getSetpoint()) <= 0.1; // TODO: Determine reasonable tolerance and move to constants file
    }

    public void setHoodSpeed(double speed) {
        this.hoodMotor.set(speed);
    }
    
}

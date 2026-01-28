// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    /** Creates a new Launcher. */
    // Here, we declare all the motors, sensors, and other objects that this subsystem needs

    // Creating launcher Motor objects
    private final TalonFX launcherMotorA = new TalonFX(53); //TODO: Replace with actual CAN ID
    private final TalonFX launcherMotorB = new TalonFX(54); //TODO: Replace with actual CAN ID
    
    // hood motor and controller objects
    private final SparkMax hoodMotor = new SparkMax(55, MotorType.kBrushless);
    private final SparkClosedLoopController hoodPIDController = hoodMotor.getClosedLoopController();
    private double hoodSetpoint = 0.0;

    public Launcher() {
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure LauncherMotorA
        configureLauncherMotorA();

        // Configure LauncherMotorB
        configureLauncherMotorB();

        // Configure hood motor and controller
        configureHoodMotor();
    }

    /* Private, internal functions */
    private void configureLauncherMotorA() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID (Velocity)
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html#velocity-control
        config.Slot0.kS = 0.0; // TODO: increase until the motors overcome friction in the launcher
        config.Slot0.kV = 0.0; // TODO: set to 0.12
        config.Slot0.kP = 0.0; // TODO: set to 1/(max speed)
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Neutral Mode
        // We typically set flywheels to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // TODO: Enable which ever is correct for this motor
        // I suggest this be set such that a positive number launcher the game piece
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying the config
        this.launcherMotorA.getConfigurator().apply(config);
    }

    private void configureLauncherMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Neutral Mode
        // We typically set flywheels to coast mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Setting the motor direction
        // TODO: Enable which ever is correct for this motor
        // I suggest this be set such that a positive number launcher the game piece
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying the config
        this.launcherMotorB.getConfigurator().apply(config);

        // set motorB to follow motorA
        this.launcherMotorB.setControl(new Follower(this.launcherMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
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
        // TODO: Change to break mode after configuring PID
        // config.idleMode(IdleMode.kBrake); 
        config.idleMode(IdleMode.kCoast); 
        
        // Setting the motor direction
        // TODO: confirm if this should be true of false. 
        // I recommend setting this such that a positive number rotates the hood away from its resting position.
        config.inverted(false);

        // setting the forward / reverse position limits so the hood doesn't rip itself apart... hopefully :)
        config.softLimit.forwardSoftLimit(0); //TODO: Change limits to based on real hardware
        config.softLimit.forwardSoftLimitEnabled(false); //TODO: Change to true to enable
        config.softLimit.reverseSoftLimit(0); //TODO: Change limits to based on real hardware (this one will probably stay at 0)
        config.softLimit.reverseSoftLimitEnabled(false); //TODO: Change to true to enable

        // configuring the pid controller for the hood angle
        config.closedLoop
        .p(0.0) // TODO: Set p to 1/(hood range)
        .i(0.0)
        .d(0.0);

        // finally, we apply our config to as persistent parameters
        this.hoodMotor.configure(config, null, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // This is a good spot to put any state-monitoring, smart dashboard outputs, logging, etc
        SmartDashboard.putNumber("Launcher/Velocity RPS", launcherMotorA.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Launcher/At Speed", flywheelIsAtVelocity());

        SmartDashboard.putNumber("HoodPose", this.hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Launcher/At Speed", hoodIsAtSetpoint());
    }

    /* Public functions for commands */
    /**
     * Sets the velocity of the launcher motors
     * @param velocityRps
     */
    public void setFlywheelVelocity(double velocityRps) {
        launcherMotorA.setControl(new VelocityVoltage(velocityRps));
    }

    /**
     * Sets the flywheel percentage in open-loop mode. Do not use for launching game pieces. This is to help with setup and testing.
     * @param speedFraction between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setFlywheelPercent(double speedFraction) {
        this.launcherMotorA.set(speedFraction);

    }

    /**
     * Stops the Launcher motors.
     */
    public void stop() {
        launcherMotorA.stopMotor();
    }

    /**
     * Returns if the launcher is at its target velocity.
     * @return True if the intake is at its set velocity, otherwise false
     */
    public boolean flywheelIsAtVelocity() {
        return Math.abs(launcherMotorA.getClosedLoopError().getValue()) <= 10; // TODO: Configure velocity tolerance, move to constants file
    }

    /**
     * Sets the hood's target position
     * @param rotations the position in the hood in rotations
     */
    public void setHoodPosition(double rotations) {
        this.hoodSetpoint = rotations;
        this.hoodPIDController.setSetpoint(rotations, SparkMax.ControlType.kPosition);
    }

    /**
     * Returns if the hood is at its set point or not.
     * @return True if the hood is at its setpoint, false otherwise.
     */
    public boolean hoodIsAtSetpoint() {
        // Unlike the TalonFX, the spark max doesn't have a function call for how close it is.
        // Therefore, we will look at the the current position, and compare it to the stored setpoint
        return Math.abs(this.hoodMotor.getEncoder().getPosition() - this.hoodSetpoint) <= 0.5; // TODO: Determine reasonable tolerance and move to constants file
    }
}

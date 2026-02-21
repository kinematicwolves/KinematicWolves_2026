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
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    /** Creates a new Launcher. */

    // Step one, create all the objects we need
    private final TalonFX launcherMotor1 = new TalonFX(41); 
    private final TalonFX launcherMotor2 = new TalonFX(42); 
    // private final TalonFX kickerMotor = new TalonFX(43);      // for now let's assume kicker motor is # 43
    private SparkMax kickerMotor = new SparkMax(43, SparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
    



    public Launcher() {
        // Step 2, apply whatever configs we need
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure LauncherMotorA
        configureLauncherMotor1();

        // Configure LauncherMotorB
        configureLauncherMotor2();

        configureKickerMotor();

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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying the config
        this.launcherMotor2.getConfigurator().apply(config);

        // set motorB to follow motorA
        this.launcherMotor2.setControl(new Follower(this.launcherMotor1.getDeviceID(), MotorAlignmentValue.Opposed ));
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
        // config.idleMode(IdleMode.kBrake); 
        config.idleMode(IdleMode.kCoast); 
        
        // Setting the motor direction
        // TODO: confirm if this should be true of false. 
        // I recommend setting this such that a positive number rotates the hood away from its resting position.
        config.inverted(false);

       
        // configuring the pid controller for the hood angle
       
        // finally, we apply our config to as persistent parameters
        this.kickerMotor.configure(config, null, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Launcher/Velocity RPS", launcherMotor1.getVelocity().getValueAsDouble());
         SmartDashboard.putNumber("Kicker/Velocity RPS", kickerEncoder.getVelocity());
    }

    /* Public functions for commands */
    /**
     * Sets the flywheel percentage in open-loop mode. Do not use for launching game pieces. This is to help with setup and testing.
     * @param speedFraction between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setFlywheelPercent(double speedFraction) {
        this.launcherMotor1.set(speedFraction);
        this.kickerMotor.set(speedFraction);
    }

    /**
     * talks to the kicker motor and sets its speed
     * @param percentage a value between -1 and 1
     */
    public void setKickerPercent(double percentage){ 
        kickerMotor.set(percentage);
    }
}

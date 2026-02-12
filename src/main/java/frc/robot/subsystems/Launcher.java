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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    /** Creates a new Launcher. */

    // Step one, create all the objects we need
    private final TalonFX launcherMotor1 = new TalonFX(41); 
    private final TalonFX launcherMotor2 = new TalonFX(42); 


    public Launcher() {
        // Step 2, apply whatever configs we need
        // Here, we will configure all the motors, and to whatever other setup we need.
        // I split this up into separate functions for readability

        // Configure LauncherMotorA
        configureLauncherMotor1();

        // Configure LauncherMotorB
        configureLauncherMotor2();

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


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
}

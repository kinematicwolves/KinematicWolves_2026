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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.LauncherProfile;
import frc.robot.ShotParams;

public class Launcher extends SubsystemBase {
    /** Creates a new Launcher. */

    // Step one, create all the objects we need
    private final TalonFX launcherMotor1 = new TalonFX(41, TunerConstants.kCANBus); //TODO: Move to constants file
    private final TalonFX launcherMotor2 = new TalonFX(42, TunerConstants.kCANBus); //TODO: Move to constants file

    // hood motor and controller objects
    private final SparkMax hoodMotor = new SparkMax(44, SparkLowLevel.MotorType.kBrushless); //TODO: Move to constants file
    private final SparkClosedLoopController hoodPIDController = hoodMotor.getClosedLoopController();

    private final InterpolatingTreeMap<Double, ShotParams> shotTable =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),ShotParams::interpolate) {{
            // distance [m] → (hood rotations, rpm)
            put(1.0, new ShotParams(1.0, 1000.0));
            put(1.2, new ShotParams(1.5, 1000.0));
            put(2.0, new ShotParams(2.0, 2000.0));
            put(2.5, new ShotParams(2.5, 2000.0));
            put(3.0, new ShotParams(3.0, 3000.0));
        }};

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

        // reset the hood position at code startup
        this.hoodMotor.getEncoder().setPosition(0);
    }

    private double getDistanceToGoal(Pose2d currentPose) {
        Transform2d bot2goal;

        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            bot2goal = new Transform2d(currentPose, LauncherProfile.redHub);
        }
        else {
            bot2goal = new Transform2d(currentPose, LauncherProfile.blueHub);
        }

        return bot2goal.getTranslation().getNorm();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("HoodPose [Rotations]",   this.hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("HoodAtSetPoint",        this.hoodIsAtSetpoint());
        SmartDashboard.putNumber("HoodTarget [Rotations]", this.hoodPIDController.getSetpoint());
        SmartDashboard.putNumber("LauncherSpeeed [RPM]",   this.launcherMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("LauncherAtSpeed",       this.flywheelAtSpeed());
    }

    /* Public functions for commands */
    /**
     * Sets the flywheel percentage in open-loop mode. Do not use for launching game pieces. This is to help with setup and testing.
     * @param percent between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setFlywheelPercent(double percent) {
        this.launcherMotor1.set(percent);
    }

    /**
     * Sets the flywheel speed in rpms using closed loop mode.
     * @param speed
     */
    public void setFlywheelSpeed(double speed) {
        this.launcherMotor1.setControl(new VelocityVoltage(speed));
    }

    /**
     * @return true if the flywheel is at its setpoint, false otherwhise
     */
    public boolean flywheelAtSpeed() {
        return Math.abs(this.launcherMotor1.getClosedLoopError().getValueAsDouble()) < 10.0; // TODO: Determine tolerance,
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
        return Math.abs(this.hoodMotor.getEncoder().getPosition() - this.hoodPIDController.getSetpoint()) <= 0.1; // TODO: Move tolerance to constants file
    }

    /**
     * Sets the hood speed percentage in open-loop mode. Do not use for launching game pieces. This is to help with setup and testing.
     * @param percent between -1 and 1. -1 is full speed in reverse, 1 is full speed forward.
     */
    public void setHoodPercent(double percent) {
        this.hoodMotor.set(percent);
    }

    public void setHoodAndSpeedFromPose(Pose2d currentPose) {
        // compute the distancet the goal
        double distanceToGoal = this.getDistanceToGoal(currentPose);
        // TODO: Handle distances outside testing range
        // TODO: Hangle fudge factor

        // use the lookup table to find our shot parameters
        ShotParams params = this.shotTable.get(distanceToGoal);

        // apply the parameters to the hood, flywheeel
        setHoodPosition(params.hoodRotations);
        setFlywheelSpeed(params.rpm);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherProfile;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.ShotParams;

public class Launcher extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX launcherMotor1 = new TalonFX(LauncherProfile.launcherMotor1CanID, TunerConstants.kCANBus);
    private final TalonFX launcherMotor2 = new TalonFX(LauncherProfile.launcherMotor2CanID, TunerConstants.kCANBus);

    private final SparkMax hoodMotor = new SparkMax(LauncherProfile.hoodMotorCanID, SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController hoodPIDController = hoodMotor.getClosedLoopController();

    // --- State & Filtering ---
    private final Debouncer debouncer = new Debouncer(0.5);
    public double speedFudgeFactor = 0.0;
    public double hoodFudgeFactor = 0.0;

    // --- Shot Logic ---
    private final InterpolatingTreeMap<Double, ShotParams> shotTable =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),ShotParams::interpolate) {{
            // distance [m] → (hood rotations, rps)
            // TODO: Refine these values on the real field
            put(1.0, new ShotParams(1.0,  50));
            put(1.2, new ShotParams(1.5, 100));
            put(2.0, new ShotParams(2.0, 150));
            put(2.5, new ShotParams(2.5, 200));
            put(3.0, new ShotParams(3.0, 300));
        }};

    public Launcher() {
        configureFlywheels();
        configureHoodMotor();
    }

    private void configureFlywheels() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID (Velocity)
        config.Slot0.kS = 0.1; // Static friction
        config.Slot0.kV = 0.12; // Velocity feedforward
        config.Slot0.kP = 0.11; // Error correction

        // Current Limits (Bumped up for Kraken flywheels)
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Neutral Mode & Direction
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Apply to Leader
        this.launcherMotor1.getConfigurator().apply(config);

        // Apply exactly the same config to Follower
        this.launcherMotor2.getConfigurator().apply(config);
        
        // Set Motor 2 to follow Motor 1
        this.launcherMotor2.setControl(new Follower(this.launcherMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureHoodMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(20);
        config.idleMode(IdleMode.kBrake); 
        config.inverted(false);

        // Soft limits to prevent the hood from ripping itself apart
        config.softLimit.forwardSoftLimit(5);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);

        // Position PID
        config.closedLoop
            .p(1.0, ClosedLoopSlot.kSlot0)
            .i(0.0, ClosedLoopSlot.kSlot0)
            .d(0.0, ClosedLoopSlot.kSlot0);

        // Apply everything at once and save to flash
        this.hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Reset the hood position at code startup
        this.hoodMotor.getEncoder().setPosition(0);
    }

    private double getDistanceToGoal(Pose2d currentPose) {
        Translation2d goalTranslation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
            ? LauncherProfile.redHub.getTranslation() 
            : LauncherProfile.blueHub.getTranslation();

        return currentPose.getTranslation().getDistance(goalTranslation);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("HoodPose [Rotations]",   this.hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("HoodAtSetPoint",        this.hoodIsAtSetpoint());
        SmartDashboard.putNumber("LauncherSpeed [RPS]",    this.launcherMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("LauncherAtSpeed",       this.flywheelAtSpeed());
    }

    /* --- Public Commands --- */

    public void setFlywheelPercent(double percent) {
        this.launcherMotor1.set(percent);
    }

    public void setFlywheelSpeed(double speed) {
        this.launcherMotor1.setControl(new VelocityVoltage(speed).withEnableFOC(true));
    }

    public void turnFlywheelOff() {
        this.launcherMotor1.setControl(new CoastOut());
    }

    public boolean flywheelAtSpeed() {
        return Math.abs(this.launcherMotor1.getClosedLoopError().getValueAsDouble()) < LauncherProfile.launcherTolerance;
    }

    public void setHoodPosition(double rotations) {
        this.hoodPIDController.setSetpoint(rotations, SparkMax.ControlType.kPosition);
    }

    public boolean hoodIsAtSetpoint() {
        return this.debouncer.calculate(Math.abs(this.hoodMotor.getEncoder().getPosition() - this.hoodPIDController.getSetpoint()) <= 0.1);
    }

    public void setHoodAndSpeedFromPose(Pose2d currentPose) {
        double distanceToGoal = this.getDistanceToGoal(currentPose);
        ShotParams params = this.shotTable.get(distanceToGoal);

        setHoodPosition(params.hoodRotations + hoodFudgeFactor);
        setFlywheelSpeed(params.rps + speedFudgeFactor);
    }
}
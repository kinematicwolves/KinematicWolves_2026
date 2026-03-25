package frc.robot.subsystems;

import java.util.function.DoubleSupplier; // Added for continuous aiming

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherProfile;
import frc.robot.generated.TunerConstants;

public class Launcher extends SubsystemBase {

    private final TalonFX m_leftFlywheel;
    private final TalonFX m_rightFlywheel;
    private final SparkMax m_hood;

    // CTRE Velocity Request
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Interpolating Maps
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();

    private double m_currentTargetRPS = 0.0;
    private double m_currentTargetHood = 0.0;

    public Launcher() {
        m_leftFlywheel = new TalonFX(LauncherProfile.kFlywheelLeftID, TunerConstants.kCANBus);
        m_rightFlywheel = new TalonFX(LauncherProfile.kFlywheelRightID, TunerConstants.kCANBus);
        m_hood = new SparkMax(LauncherProfile.kHoodID, MotorType.kBrushless);

        setupInterpolation();
        configureHardware();
    }

    private void setupInterpolation() {
        for (double[] data : LauncherProfile.kShootingData) {
            m_rpmMap.put(data[0], data[1]);
            m_hoodMap.put(data[0], data[2]);
        }
    }

    private void configureHardware() {
        /* --- KRAKEN FLYWHEELS --- */
        TalonFXConfiguration flyConfig = new TalonFXConfiguration();
        flyConfig.Slot0.kP = LauncherProfile.kFlywheelP;
        flyConfig.Slot0.kV = LauncherProfile.kFlywheelV;
        flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        flyConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftFlywheel.getConfigurator().apply(flyConfig);

        flyConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightFlywheel.getConfigurator().apply(flyConfig);

        /* --- HOOD --- */
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30); 
        
        // Use the imported FeedbackSensor enum directly
        hoodConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(LauncherProfile.kHoodP);

        // Apply and Persist
        m_hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ========================================================= */
    /* LOGIC METHODS                                             */
    /* ========================================================= */

    public void runShooter(double targetRPS, double targetHoodRotations) {
        m_currentTargetRPS = targetRPS;
        m_currentTargetHood = targetHoodRotations;

        m_leftFlywheel.setControl(m_velocityRequest.withVelocity(targetRPS));
        m_rightFlywheel.setControl(m_velocityRequest.withVelocity(targetRPS));
        
        m_hood.getClosedLoopController().setSetpoint(targetHoodRotations, SparkMax.ControlType.kPosition);
    }

    public void stop() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
        m_hood.stopMotor();

        // Reset targets when stopped so isReadyToFire doesn't return true accidentally
        m_currentTargetRPS = 0.0; 
        m_currentTargetHood = 0.0;
    }

    /**
     * Checks if the shooter is rev'd up and the hood is in position based on the LIVE target.
     */
    public boolean isReadyToFire() {
        if (m_currentTargetRPS == 0.0) return false; // Not trying to shoot

        double leftErr = Math.abs(m_leftFlywheel.getVelocity().getValueAsDouble() - m_currentTargetRPS);
        double hoodErr = Math.abs(m_hood.getEncoder().getPosition() - m_currentTargetHood);
        return (leftErr < LauncherProfile.kRPSTolerance) && (hoodErr < LauncherProfile.kHoodTolerance);
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /**
     * Takes a distance supplier so the RPS and Angle update LIVE as the robot drives.
     */
    public Command continuousAimCommand(DoubleSupplier distanceMetersSupplier) {
        return run(() -> {
            double distance = distanceMetersSupplier.getAsDouble();
            if (distance > 0.1) { // If we have a valid target
                double rps = m_rpmMap.get(distance);
                double hood = m_hoodMap.get(distance);
                runShooter(rps, hood);
            } else {
                // Default to a known safe shot if vision drops
                runShooter(m_rpmMap.get(1.0), m_hoodMap.get(1.0));
            }
        }).finallyDo(this::stop).withName("ContinuousAim");
    }

    /**
     * Fallback "Hub Shot" manually forced by the driver.
     */
    public Command hubShotCommand() {
        // Pulls the 1.0 meter mapped values
        return run(() -> runShooter(m_rpmMap.get(1.0), m_hoodMap.get(1.0)))
               .finallyDo(this::stop)
               .withName("HubShot");
    }

    public static Command closeShotCommand(Launcher launcher, Indexer indexer) {
        return Commands.parallel(
            launcher.hubShotCommand(),
            Commands.waitUntil(launcher::isReadyToFire)
                    .andThen(indexer.feedShooterCommand())
        ).withName("FenderShotSequence");
    }

    /**
     * Reads RPS and Hood setpoints directly from the SmartDashboard.
     * Great for finding the "Goldilocks" settings at a new distance.
     */
    public Command technicianTuningCommand() {
        return run(() -> {
            double rps = SmartDashboard.getNumber("Tech/Target RPS", 0.0);
            double hood = SmartDashboard.getNumber("Tech/Target Hood", 0.0);
            runShooter(rps, hood);
        }).finallyDo(this::stop).withName("TechnicianTuning");
    }

    @Override
    public void periodic() {
        // Standard Telemetry
        SmartDashboard.putNumber("Launcher/Left RPS", m_leftFlywheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Launcher/Hood Pos", m_hood.getEncoder().getPosition());
        SmartDashboard.putBoolean("Launcher/ReadyToFire", isReadyToFire());

        // --- TECHNICIAN INPUTS ---
        // setDefaultNumber only sets the value if the key doesn't exist yet (preserves what the tech typed)
        SmartDashboard.setDefaultNumber("Technician/Target RPS", 0.0);
        SmartDashboard.setDefaultNumber("Technician/Target Hood", 0.0);
    }
}
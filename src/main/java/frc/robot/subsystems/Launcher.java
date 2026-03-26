package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.generated.TunerConstants;

public class Launcher extends SubsystemBase {

    private final TalonFX m_leftFlywheel;
    private final TalonFX m_rightFlywheel;
    private final SparkMax m_hood;

    // CTRE closed-loop velocity control object
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Dynamic Look-up Tables: [Distance] -> [RPS or Hood Angle]
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();

    private double m_currentTargetRPS = 0.0;
    private double m_currentTargetHood = 0.0;

    // Prevents "flickering" ready state; must stay in tolerance for 0.5s to be "ready"
    Debouncer debouncer = new Debouncer(0.5);

    public Launcher() {
        m_leftFlywheel = new TalonFX(LauncherProfile.kFlywheelLeftID, TunerConstants.kCANBus);
        m_rightFlywheel = new TalonFX(LauncherProfile.kFlywheelRightID, TunerConstants.kCANBus);
        m_hood = new SparkMax(LauncherProfile.kHoodID, MotorType.kBrushless);

        setupInterpolation();
        configureHardware();
    }

    /** Loads data points from Constants.java into the interpolation maps */
    private void setupInterpolation() {
        for (double[] data : LauncherProfile.kShootingData) {
            m_rpmMap.put(data[0], data[1]);
            m_hoodMap.put(data[0], data[2]);
        }
    }

    /** Sets up PID, Inversions, and Brake modes */
    private void configureHardware() {
        TalonFXConfiguration flyConfig = new TalonFXConfiguration();
        flyConfig.Slot0.kP = LauncherProfile.kFlywheelP;
        flyConfig.Slot0.kV = LauncherProfile.kFlywheelV;
        flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Let flywheels spin down naturally
        
        // Motors spin opposite directions to launch the ball
        flyConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftFlywheel.getConfigurator().apply(flyConfig);

        flyConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightFlywheel.getConfigurator().apply(flyConfig);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig
            .idleMode(IdleMode.kBrake) // Hold position when not moving
            .smartCurrentLimit(30); 
        
        hoodConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(LauncherProfile.kHoodP);

        m_hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Sends velocity and position targets to motor controllers */
    public void runShooter(double targetRPS, double targetHoodRotations) {
        m_currentTargetRPS = targetRPS;
        m_currentTargetHood = targetHoodRotations;

        m_leftFlywheel.setControl(m_velocityRequest.withVelocity(targetRPS));
        m_rightFlywheel.setControl(m_velocityRequest.withVelocity(targetRPS));
        
        m_hood.getClosedLoopController().setSetpoint(targetHoodRotations, SparkMax.ControlType.kPosition);
    }

    /** Zeroes all motor outputs */
    public void stop() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
        m_hood.stopMotor();
        m_currentTargetRPS = 0.0; 
        m_currentTargetHood = 0.0;
    }

    /** Returns true if current RPS and Hood Angle match targets (with debounce) */
    public boolean isReadyToFire() {
        if (m_currentTargetRPS == 0.0) return false; 

        double leftErr = Math.abs(m_leftFlywheel.getVelocity().getValueAsDouble() - m_currentTargetRPS);
        double hoodErr = Math.abs(m_hood.getEncoder().getPosition() - m_currentTargetHood);
        
        return debouncer.calculate((leftErr < LauncherProfile.kRPSTolerance) && (hoodErr < LauncherProfile.kHoodTolerance));
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /** Automatically adjusts shooter speed/angle based on live distance */
    public Command continuousAimCommand(DoubleSupplier distanceMetersSupplier) {
        return run(() -> {
            double distance = distanceMetersSupplier.getAsDouble();
            if (distance > 0.1) {
                runShooter(m_rpmMap.get(distance), m_hoodMap.get(distance));
            } else {
                runShooter(m_rpmMap.get(1.0), m_hoodMap.get(1.0)); // Default safe shot
            }
        }).finallyDo(this::stop).withName("ContinuousAim");
    }

    /** Fires from against the goal using hardcoded distance values */
    public static Command closeShotCommand(Launcher launcher, Indexer indexer, Intake intake) {
        return Commands.parallel(
            launcher.run(() -> launcher.runShooter(launcher.m_rpmMap.get(1.0), launcher.m_hoodMap.get(1.0))),
            
            // Wait for spool, then feed; pauses feed if RPS drops
            Commands.sequence(
                Commands.waitUntil(launcher::isReadyToFire),
                indexer.feedShooterCommand()
                       .alongWith(intake.runRollersCommand(IntakeProfile.kRollerVoltage))
                       .onlyWhile(launcher::isReadyToFire) 
            ).repeatedly()
        ).finallyDo(launcher::stop).withName("FenderShotSequence");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Left RPS", m_leftFlywheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Launcher/Hood Pos", m_hood.getEncoder().getPosition());
        SmartDashboard.putBoolean("Launcher/ReadyToFire", isReadyToFire());
    }
}
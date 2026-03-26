package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier; // <-- ADDED

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveProfile;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Manages the Swerve Drivetrain hardware and high-level movement logic.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms for simulation
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Telemetry m_logger = new Telemetry(SwerveProfile.kMaxSpeed);

    // Operator Perspective: Ensures "Forward" is always away from the driver station
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private Alliance m_lastAlliance = null;

    /** PathPlanner request for robot-relative speeds */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Teleop request for field-relative movement */
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1) 
            .withRotationalDeadband(0.1);

    /** Locks the wheels into an X shape to prevent being pushed */
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // <-- ADDED

    // SysId routines for motor characterization (tuning gains)
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    
    public final Field2d field2d = new Field2d(); // Visualizer for Dashboard
    
    // SysId routine setups...
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this)
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        this.registerTelemetry(m_logger::telemeterize); 
    }

    /** Connects this drivetrain to PathPlanner Lib for autonomous paths */
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose, // Current position
                this::resetPose,      // Reset position
                () -> getState().Speeds, // Current velocity
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    SwerveProfile.kTranslationPID,
                    SwerveProfile.kRotationPID 
                ),
                config,
                () -> false, // Do not flip path (using Red/Blue specific paths instead)
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config", ex.getStackTrace());
        }
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /** Standard field-oriented drive command */
    public Command applyDrive(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier rotationalRate) {
        return run(() -> this.setControl(
            m_driveRequest
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withRotationalRate(rotationalRate.getAsDouble())
        )).withName("StandardDrive");
    }

    /** Field-oriented drive with speed reduction for precision aiming */
    public Command applySlowDrive(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier visionRotationalRate) {
        return run(() -> this.setControl(
            m_driveRequest
                .withVelocityX(velocityX.getAsDouble() * SwerveProfile.kSlowTranslationScalar)
                .withVelocityY(velocityY.getAsDouble() * SwerveProfile.kSlowTranslationScalar)
                .withRotationalRate(visionRotationalRate.getAsDouble())
        )).withName("SlowDrive");
    }

    /** Resets the gyro heading to 0 (facing Red Wall) */
    public Command resetHeading() {
        return runOnce(() -> this.seedFieldCentric()).withName("ResetHeading");
    }

    /** Takes any Phoenix 6 SwerveRequest and applies it continuously */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /** Dedicated command to lock the wheels into an X shape */
    public Command applyBrake() {
        return run(() -> this.setControl(brake)).withName("Brake");
    }

    @Override
    public void periodic() {
        // Update operator forward direction based on current Alliance
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != m_lastAlliance) {
            m_lastAlliance = currentAlliance.get();
            setOperatorPerspectiveForward(
                m_lastAlliance == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
        }
        
        // Update dashboard field map
        field2d.setRobotPose(this.getState().Pose);
        SmartDashboard.putData("Field", field2d);
    }

    /** Adds vision data to the internal pose estimator */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // High X/Y trust (0.7), but basically ignore vision rotation (9999999).
        // We ignore rotation because the Gyro is much more stable than camera estimates.
        super.addVisionMeasurement(
            visionRobotPoseMeters, 
            Utils.fpgaToCurrentTime(timestampSeconds),
            VecBuilder.fill(0.7, 0.7, 9999999) 
        );
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    // Boilerplate/Sim code...
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
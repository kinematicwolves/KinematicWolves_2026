package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType; // <-- ADDED IMPORT
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Telemetry m_logger = new Telemetry(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude());

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    
    private Alliance m_lastAlliance = null;

    // --- THE FIX: DEDICATED PATHPLANNER VELOCITY CONTROL ---
    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity); // <-- FORCES ACTIVE BRAKING AT WAYPOINTS

    /** Standard Teleop Drive Request */
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1) 
            .withRotationalDeadband(0.1);

    /** Dedicated request for auto-aiming. Rotational Deadband is REMOVED so small PID corrections are not ignored. */
    private final SwerveRequest.FieldCentric m_aimRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1) 
            .withRotationalDeadband(0.0);

    /** Locks the wheels into an X shape to prevent being pushed */
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    public final Field2d field2d = new Field2d();
    
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
        new SysIdRoutine.Mechanism(
            output -> {
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

    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        this.registerTelemetry(m_logger::telemeterize); 
    }

    public Swerve(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
        configureAutoBuilder();
        this.registerTelemetry(m_logger::telemeterize); 
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
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
                () -> false, // No flipping, per your "sketch red/blue" strategy
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config", ex.getStackTrace());
        }
    }

    /* ========================================================= */
    /* CUSTOM LOGIC: COMMAND FACTORIES FOR DRIVING & SLOW MODE   */
    /* ========================================================= */

    public Command applyDrive(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier rotationalRate) {
        return run(() -> this.setControl(
            m_driveRequest
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withRotationalRate(rotationalRate.getAsDouble())
        )).withName("StandardDrive");
    }

    public Command applySlowDrive(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier visionRotationalRate) {
        return run(() -> {
            double aimRate = visionRotationalRate.getAsDouble();
            SmartDashboard.putNumber("Vision/Aim Rate Sent to Swerve", aimRate);

            this.setControl(
                m_aimRequest
                    .withVelocityX(velocityX.getAsDouble() * SwerveProfile.kSlowTranslationScalar)
                    .withVelocityY(velocityY.getAsDouble() * SwerveProfile.kSlowTranslationScalar)
                    .withRotationalRate(aimRate)
            );
        }).withName("SlowDrive");
    }

    public Command resetHeading() {
        return runOnce(() -> this.seedFieldCentric()).withName("ResetHeading");
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command applyBrake() {
        return run(() -> this.setControl(brake)).withName("Brake");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != m_lastAlliance) {
            m_lastAlliance = currentAlliance.get();
            setOperatorPerspectiveForward(
                m_lastAlliance == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
        }
        
        field2d.setRobotPose(this.getState().Pose);
        SmartDashboard.putData("Field", field2d);
    }

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

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, 
            Utils.fpgaToCurrentTime(timestampSeconds),
            VecBuilder.fill(0.7, 0.7, 9999999) 
        );
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }
}
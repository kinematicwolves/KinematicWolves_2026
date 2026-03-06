// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeProfile;
import frc.robot.commands.FeedWithSpeeds;
import frc.robot.commands.IntakeToPose;
import frc.robot.commands.IntakeWithSpeeds;
import frc.robot.commands.LaunchFromPose;
import frc.robot.commands.LaunchWithParams;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lighting;

public class RobotContainer {
    // this is swerve stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController opController     = new CommandXboxController(1);

    /* subsystems */
    public  final Lighting lighting = new Lighting();
    private final Launcher launcher = new Launcher();
    private final Indexer  indexer  = new Indexer();
    private final Intake   intake   = new Intake();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* custom triggers */
    private Trigger hubActiveSoon = new Trigger(() -> isHubActive(5) == true); // true when the hub is active in 5 seconds
    private Trigger hubActive     = new Trigger(() -> isHubActive(0) == true); // true when the hub is active

    /* some variables */
    public double launcherSpeed = 0;
    public double launcherAngle = 0;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // TODO: choose a default auto that actually exists plz
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        /* Named commands for path planner here */
        NamedCommands.registerCommand("deployIntake",   new IntakeToPose(intake, IntakeProfile.deployPose, IntakeProfile.gentleSlot));
        NamedCommands.registerCommand("retractIntake",  new IntakeToPose(intake, IntakeProfile.zeroPose,   IntakeProfile.aggressiveSlot));
        NamedCommands.registerCommand("ingest",         new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakeSpeed, 0.6));
        NamedCommands.registerCommand("launchFromPose", new LaunchFromPose(drivetrain, launcher, indexer, 0.6, 1));

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        setupPathplannerLogging();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    .withRotationalDeadband(0.1 * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);

        /* Driver controls */
        // launcher stuff
        driverController.rightTrigger(0.5)
            .onTrue(new LaunchWithParams(launcher, this))
            .whileTrue(
                drivetrain.applyRequest(() -> fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward under driver control
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left / right with driver control
                    .withRotationalRate(-(LimelightHelpers.getTX("limelight") * 0.05 * MaxAngularRate)))); // rotate from limelight value

        driverController.b()
            .onTrue(new InstantCommand(() -> {this.launcherAngle=0; this.launcherSpeed=0;}));

        driverController.rightBumper()
            .whileTrue(new FeedWithSpeeds(indexer, 0.6, 1));

        // intake stuff
        driverController.leftTrigger(0.5)
            .onTrue(new IntakeToPose(intake, IntakeProfile.deployPose, IntakeProfile.gentleSlot))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakeSpeed, 0));

        driverController.leftBumper()
            .onTrue(new IntakeToPose(intake, IntakeProfile.zeroPose, IntakeProfile.aggressiveSlot))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, IntakeProfile.retractSpeed, 0)); // helps the intake go up

        /* operator controls */
        opController.povUp()
            .onTrue(new InstantCommand(() -> {BumpLauncherSpeed(10);}));
        opController.povDown()
            .onTrue(new InstantCommand(() -> {BumpLauncherSpeed(-1);}));
         opController.y()
            .onTrue(new InstantCommand(() -> {BumpLauncherAngle(1);}));
        opController.a()
            .onTrue(new InstantCommand(() -> {BumpLauncherAngle(-0.25);}));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private void setupPathplannerLogging() {
        // from https://pathplanner.dev/pplib-custom-logging.html
        Field2d field = drivetrain.field2d;

        // this portion is already setup in the command swerve drivetrain
        // // Logging callback for current robot pose
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     // Do whatever you want with the pose here
        //     field.setRobotPose(pose);
        // });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
    }

    /**
     * Tells us if the hub is active, or active in earlySeconds. This can be used to let robot warm up its launcher
     * @param earlySeconds the number of seconds to turn on early
     * @return true if the hub is active or active in earlySeconds
     */
    public boolean isHubActive(double earlySeconds) {
        // Lovingly stolen (and modified) from wpi-lib: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        earlySeconds = Math.max(0.0, earlySeconds);

        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty())
            return false;

        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled())
            return true;

        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled())
            return false;

        // We're teleop enabled, compute.
        // NOTE: earlySeconds shifts all activation windows earlier in match time.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        // If we have no game data, we cannot compute, assume hub is active,
        // as it's likely early in teleop.
        if (gameData.isEmpty())
            return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift 1 is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // All shift thresholds are offset earlier by earlySeconds.
        if (matchTime > 130 - earlySeconds)
            // Transition shift, hub is active (early if earlySeconds > 0).
            return true;
        else if (matchTime > 105 - earlySeconds)
            // Shift 1 (early if earlySeconds > 0).
            return shift1Active;
        else if (matchTime > 80 - earlySeconds)
            // Shift 2 (early if earlySeconds > 0).
            return !shift1Active;
        else if (matchTime > 55 - earlySeconds)
            // Shift 3 (early if earlySeconds > 0).
            return shift1Active;
        else if (matchTime > 30 - earlySeconds)
            // Shift 4 (early if earlySeconds > 0).
            return !shift1Active;
        else
            // End game, hub always active (also reached earlier if earlySeconds > 0).
            return true;
    }

    public void BumpLauncherSpeed(double increment) {
        this.launcherSpeed += increment;
    }
    public void BumpLauncherAngle(double increment) {
        this.launcherAngle += increment;
    }
    public double getLauncherSpeed() {
        return launcherSpeed;
    }
    public double getLauncherAngle() {
        return launcherAngle;
    }
}

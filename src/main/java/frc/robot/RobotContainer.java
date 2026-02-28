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
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FeedWithSpeeds;
import frc.robot.commands.IntakeToPose;
import frc.robot.commands.IntakeWithSpeeds;
import frc.robot.commands.LaunchFromPose;
import frc.robot.commands.SetLaunchParameters;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
    // this is swerve stuff
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // subsystems
    private final Launcher launcher = new Launcher();
    private final Intake   intake   = new Intake();
    private final Indexer  indexer  = new Indexer();

    private final SendableChooser<Command> autoChooser;

    // custom triggers
    private Trigger hubActiveSoon = new Trigger(() -> isHubActive(5) == true); // true when the hub is active in 5 seconds
    private Trigger hubActive     = new Trigger(() -> isHubActive(0) == true); // true when the hub is active

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on y button press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* Intake control */
        // deploy the intake, and intake game pieces
        driverController.leftTrigger(0.5)
            .onTrue(new IntakeToPose(intake, 3.9, 1))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, 0.6, 0.3));

        // retract intake
        driverController.leftBumper().onTrue(new IntakeToPose(intake, 0, 0));
        
        /* launcher control */
        hubActiveSoon
            // launcher auto-warm-up when the hub is active soon
            .onTrue(new SetLaunchParameters(launcher, 0, 0.5));

        // point at the goal with right trigger
        driverController.rightTrigger(0.5).and(hubActive)
            .whileTrue(
                new ParallelCommandGroup(
                    new LaunchFromPose(drivetrain, launcher, indexer, 0.6, 1),
                    drivetrain.applyRequest(() -> 
                        fieldCentricDrive
                            .withRotationalRate(LimelightHelpers.getTX("limelight") * 0.5 * MaxAngularRate) // Use LL to auto rotate to goal
                            .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-driverController.getLeftX() * MaxSpeed))));

        // launch with fixed values
        driverController.a()
            .onTrue(new SetLaunchParameters(launcher, 3, 0.5));

        // feed balls to the launcher
        driverController.rightBumper()
            .whileTrue(new FeedWithSpeeds(indexer, 0.6, 1));

        // turn off launcher and feeder
        driverController.b()
            .onFalse(new SetLaunchParameters(launcher, 0, 0));
    }

    public Command getAutonomousCommand() {
       /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    //   return new InstantCommand();
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
}

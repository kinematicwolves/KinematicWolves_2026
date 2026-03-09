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
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.Constants.LauncherProfile;
import frc.robot.commands.IntakeToPose;
import frc.robot.commands.IntakeWithSpeeds;
import frc.robot.commands.LaunchFromPose;
import frc.robot.commands.LaunchFromSettings;
import frc.robot.commands.LaunchwithParams;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lighting;

public class RobotContainer {
    // this is swerve stuff
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController driverController = new CommandXboxController(0);   
    public final CommandXboxController opController     = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // subsystems
    public final Launcher launcher = new Launcher();
    public final Intake   intake   = new Intake();
    public final Indexer  indexer  = new Indexer();
    public final Lighting lighting = new Lighting();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* custom triggers */
    private Trigger hubActiveSoon = new Trigger(() -> isHubActive(5) == true); // true when the hub is active in 5 seconds
    private Trigger hubActive     = new Trigger(() -> isHubActive(0) == true); // true when the hub is active
    
    /* some variables */
    public double launcherSpeed = 65;
    public double launcherAngle = 1.5;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // TODO: choose a default auto that actually exists plz
        SmartDashboard.putData("Auto Mode", autoChooser);

        // create command for path planner
        NamedCommands.registerCommand("deployIntake",     new IntakeToPose(intake, IntakeProfile.deployedPose, IntakeProfile.gentleSlot));
        NamedCommands.registerCommand("retractIntake",    new IntakeToPose(intake, IntakeProfile.retractedPose, IntakeProfile.aggressiveSlot));
        NamedCommands.registerCommand("intakeLemons",     new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0));
        NamedCommands.registerCommand("LaunchFromCenter", new LaunchwithParams(launcher, indexer, 65, 1.5));

        configureBindings();

        /* Named commands for path planner here */
        NamedCommands.registerCommand("deployIntake",   new IntakeToPose(intake, IntakeProfile.deployedPose,  IntakeProfile.gentleSlot));
        NamedCommands.registerCommand("retractIntake",  new IntakeToPose(intake, IntakeProfile.retractedPose, IntakeProfile.aggressiveSlot));
        NamedCommands.registerCommand("ingest",         new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0.6));
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
            .onTrue(new LaunchFromSettings(launcher, indexer, this, driverController.getHID()))
            .whileTrue(
                drivetrain.applyRequest(() -> robotCentricDrive
                    .withVelocityX(-(robot2goal().getTranslation().getNorm() - LauncherProfile.idealLaunchDist) * 2 * MaxSpeed)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate((robot2goal().getRotation().getDegrees()) * 0.01 * MaxAngularRate)
            ));

        driverController.b()
            .onTrue(new InstantCommand(() -> {this.launcherAngle=0; this.launcherSpeed=0;}));
        driverController.x()
            .onTrue(new InstantCommand(() -> {this.launcherAngle=1.5; this.launcherSpeed=65;}));


        // intake stuff
        driverController.leftTrigger(0.5)
            .onTrue(new IntakeToPose(intake, IntakeProfile.deployedPose, IntakeProfile.gentleSlot))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0));
        
        driverController.leftBumper()
            .onTrue(new IntakeToPose(intake, IntakeProfile.retractedPose, IntakeProfile.aggressiveSlot))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, IntakeProfile.retractPrecent, 0)); // helps the intake go up
        
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

    public void BumpLauncerSpeed(double percent) {
        this.launcherSpeed += percent;
    }
    public void BumpLauncerAngle(double incrmenent) {
        this.launcherAngle += incrmenent;
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

    /**
     * Computes the transform of the robot to the goal, depending on the alliance of the robot
     * @return robot2goal transform. 
     * robot2goal.getTranslation() is the xy offset between the robot center and the goal
     * robot2goal.getRotation() is the angular offset between the robot's current direction and the line from the robot position to the goal.
     * robot2goal.getTranslation().getNorm() is the distance to the goal.
     */
    public Transform2d robot2goal() {
        // get the current robot position from the drivetrain
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // lookup the goal based on which alliance we are, compute the transform to that goal
        Translation2d robot2goalTranslation;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) ==  Alliance.Blue)
            robot2goalTranslation = new Transform2d(LauncherProfile.blueHub, robotPose).getTranslation();
        else
            robot2goalTranslation = new Transform2d(LauncherProfile.redHub, robotPose).getTranslation();

        // the rotation of the goal is arbitrary, we want to find the angle from the robot's current direction to the goal

        // This this gets us the angle of the vector from the robot to the goal, in field coordinates
        double vectorToGoal = Math.atan2(robot2goalTranslation.getY(), robot2goalTranslation.getX());

        // the value we care about is the difference between the current robot angle and the angle to the goal.
        // That is how much the robot must rotate by
        Rotation2d angleToGoal = new Rotation2d(vectorToGoal - robotPose.getRotation().getRadians());

        return new Transform2d(robot2goalTranslation, angleToGoal);        
    }
}

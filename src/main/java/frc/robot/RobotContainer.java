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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.commands.AutoShoot;
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
    // --- Swerve Drivetrain Settings ---
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // --- Controllers ---
    public final CommandXboxController driverController = new CommandXboxController(0);   
    public final CommandXboxController opController     = new CommandXboxController(1);

    // --- Subsystems ---
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Launcher launcher = new Launcher();
    public final Intake   intake   = new Intake();
    public final Indexer  indexer  = new Indexer();
    public final Lighting lighting = new Lighting();

    // --- Path Follower ---
    private final SendableChooser<Command> autoChooser;

    // --- Custom Triggers ---
    private Trigger hubActiveSoon = new Trigger(() -> isHubActive(5) == true);
    private Trigger hubActive     = new Trigger(() -> isHubActive(0) == true);
    
    // --- State Variables (TODO: Move to Launcher Subsystem later) ---
    public double launcherSpeed = 65;
    public double launcherAngle = 1.5;

    // --- Auto Aim PID ---
    private final ProfiledPIDController autoAimPid = new ProfiledPIDController(
        4.0, 0.0, 0.2, // TODO: Tune these! Start with P=4.0, I=0, D=0.2
        new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate * 2) 
    );

    public RobotContainer() {
        // Prevents the robot from doing a 360 to go from 179 degrees to -179 degrees
        autoAimPid.enableContinuousInput(-Math.PI, Math.PI); 

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // --- Register PathPlanner Named Commands (Duplicates Removed) ---
        NamedCommands.registerCommand("deployIntake",     new IntakeToPose(intake, IntakeProfile.deployedPose, IntakeProfile.gentleSlot));
        NamedCommands.registerCommand("retractIntake",    new IntakeToPose(intake, IntakeProfile.retractedPose, IntakeProfile.aggressiveSlot));
        NamedCommands.registerCommand("ingest",           new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0.6));
        NamedCommands.registerCommand("launchFromPose",   new LaunchFromPose(drivetrain, launcher, indexer, 0.6, 1));
        NamedCommands.registerCommand("LaunchFromCenter", new LaunchwithParams(launcher, indexer, 65, 1.5));
        NamedCommands.registerCommand("AutoShoot", new AutoShoot(launcher, indexer, drivetrain));
        
        // This command exists in your auto map but might be meant as a duplicate of ingest. 
        // Keeping it registered here just in case an old auto path uses it.
        NamedCommands.registerCommand("intakeLemons",     new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0));

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        setupPathplannerLogging();
    }

    private void configureBindings() {
        // --- Default Drivetrain Command ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
                    .withRotationalDeadband(0.1 * MaxAngularRate)
            )
        );

        // Idle while disabled to apply neutral mode
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /* --- DRIVER CONTROLS --- */
        
        // Launcher & Auto-Aim (Overhauled)
        driverController.rightTrigger(0.5)
            .onTrue(new LaunchFromSettings(launcher, indexer, this, driverController.getHID()))
            .whileTrue(
                drivetrain.applyRequest(() -> {
                    double forward = -driverController.getLeftY() * MaxSpeed;
                    double strafe = -driverController.getLeftX() * MaxSpeed;

                    // Calculate rotational speed to snap to target
                    double targetAngleRadians = getAngleToGoal().getRadians();
                    double currentAngleRadians = drivetrain.getState().Pose.getRotation().getRadians();
                    
                    double rotationRate = autoAimPid.calculate(currentAngleRadians, targetAngleRadians);

                    return fieldCentricDrive
                        .withVelocityX(forward)
                        .withVelocityY(strafe)
                        .withRotationalRate(rotationRate);
                })
            );

        driverController.b().onTrue(new InstantCommand(() -> this.launcher.turnFlywheelOff()));
        driverController.x().onTrue(new InstantCommand(() -> {this.launcherAngle=1.5; this.launcherSpeed=65;}));

        // Intake (Fixed Command Conflicts using Commands.parallel)
        driverController.leftTrigger(0.5).whileTrue(
            Commands.parallel(
                new IntakeToPose(intake, IntakeProfile.deployedPose, IntakeProfile.gentleSlot),
                new IntakeWithSpeeds(intake, indexer, IntakeProfile.intakePercent, 0)
            )
        );
        
        driverController.leftBumper().whileTrue(
            Commands.parallel(
                new IntakeToPose(intake, IntakeProfile.retractedPose, IntakeProfile.aggressiveSlot),
                new IntakeWithSpeeds(intake, indexer, IntakeProfile.retractPrecent, 0)
            )
        );
        
        /* --- OPERATOR CONTROLS --- */
        opController.povUp().onTrue(new InstantCommand(() -> BumpLauncherSpeed(10)));
        opController.povDown().onTrue(new InstantCommand(() -> BumpLauncherSpeed(-1)));
        opController.y().onTrue(new InstantCommand(() -> BumpLauncherAngle(1)));
        opController.a().onTrue(new InstantCommand(() -> BumpLauncherAngle(-0.25)));

        // Driver presses right bumper to automatically spool, feed, and fire based on their position
        driverController.rightBumper().onTrue(new AutoShoot(launcher, indexer, drivetrain)
);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // --- Logging & Telemetry ---
    private void setupPathplannerLogging() {
        Field2d field = drivetrain.field2d;
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
    }

    // --- State Management Helpers ---
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
     * Computes the exact angle required to point the robot at the alliance goal
     */
    public Rotation2d getAngleToGoal() {
        Pose2d robotPose = drivetrain.getState().Pose;
        
        Translation2d goalTranslation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
            ? LauncherProfile.redHub.getTranslation() 
            : LauncherProfile.blueHub.getTranslation();

        // Subtracting the robot's position from the goal's position creates a vector pointing exactly at the goal
        Translation2d robotToGoalVector = goalTranslation.minus(robotPose.getTranslation());
        
        // Extract the exact angle from that vector
        return robotToGoalVector.getAngle();        
    }

    /**
     * Computes the transform of the robot to the goal, depending on the alliance of the robot
     */
    public Transform2d robot2goal() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robot2goalTranslation;
        
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            robot2goalTranslation = new Transform2d(LauncherProfile.blueHub, robotPose).getTranslation();
        } else {
            robot2goalTranslation = new Transform2d(LauncherProfile.redHub, robotPose).getTranslation();
        }

        double vectorToGoal = Math.atan2(robot2goalTranslation.getY(), robot2goalTranslation.getX());
        Rotation2d angleToGoal = new Rotation2d(vectorToGoal - robotPose.getRotation().getRadians());

        return new Transform2d(robot2goalTranslation, angleToGoal);        
    }

    /**
     * Tells us if the hub is active, or active in earlySeconds.
     */
    public boolean isHubActive(double earlySeconds) {
        earlySeconds = Math.max(0.0, earlySeconds);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130 - earlySeconds) return true;
        else if (matchTime > 105 - earlySeconds) return shift1Active;
        else if (matchTime > 80 - earlySeconds) return !shift1Active;
        else if (matchTime > 55 - earlySeconds) return shift1Active;
        else if (matchTime > 30 - earlySeconds) return !shift1Active;
        else return true;
    }
}
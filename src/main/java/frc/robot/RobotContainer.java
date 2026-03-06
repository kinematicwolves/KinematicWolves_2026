// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LauncherProfile;
import frc.robot.commands.FeedWithSpeeds;
import frc.robot.commands.IntakeToPose;
import frc.robot.commands.IntakeWithSpeeds;
import frc.robot.commands.LaunchwithParams;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
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
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)    
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);   
    private final CommandXboxController opController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // subsystems
    public final Launcher launcher = new Launcher();
    private final Intake   intake   = new Intake();
    private final Indexer  indexer  = new Indexer();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    // some variables
    public double launcherPercent = 0;
    public double launcherAngle   = 0;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // create command for path planner
        // NamedCommands.registerCommand("deployIntake",  new IntakeToPose(intake, Constants.IntakeProfile.deployedPose,  0));
        // NamedCommands.registerCommand("retractIntake", new IntakeToPose(intake, Constants.IntakeProfile.retractedPose, 0));
        // NamedCommands.registerCommand("ingeestLemons", new IntakeWithSpeeds(intake, indexer, 0.6, 0));

        configureBindings();

        /*Named Commands For PathPlanner */

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
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Reset the field-centric heading on left bumper press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);

        // launcher stuff
        driverController.rightTrigger(0.5)
            .onTrue(new LaunchwithParams (launcher, this, getLauncherPercent(), getLauncherAngle()))
            .whileTrue(
                drivetrain.applyRequest(() -> fieldCentricDrive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-(LimelightHelpers.getTX("limelight")*0.05*MaxAngularRate)))); // rotate from limelight value

        driverController.b()
            .onTrue(new InstantCommand(() -> {this.launcherAngle=0; this.launcherPercent=0;}).andThen(new LaunchwithParams(launcher, this, 0, 0)));
        
        driverController.rightBumper()
            .whileTrue(new FeedWithSpeeds(indexer, 0.6, 1));

        // intake stuff
        driverController.leftTrigger(0.5)
            .onTrue(new IntakeToPose(intake, 3.9, 0))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, 1, 0));
        
        driverController.leftBumper()
            .onTrue(new IntakeToPose(intake, 0, 1))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, -0.5, 0)); // helps the intake go up
        
        /* operator controls */
        opController.povUp()
            .onTrue(new InstantCommand(() -> {BumpLauncerPercent(10);}));
        opController.povDown()
            .onTrue(new InstantCommand(() -> {BumpLauncerPercent(-1);}));
         opController.y()
            .onTrue(new InstantCommand(() -> {BumpLauncerAngle(0.5);}));
        opController.a()
            .onTrue(new InstantCommand(() -> {BumpLauncerAngle(-0.5);}));
    }

    public Command getAutonomousCommand() {
       /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void BumpLauncerPercent(double percent) {
        this.launcherPercent += percent;
    }
    public void BumpLauncerAngle(double incrmenent) {
        this.launcherAngle += incrmenent;
    }
    public double getLauncherPercent() {
        return launcherPercent;
    }
    public double getLauncherAngle() {
        return launcherAngle;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;
import frc.robot.commands.FeedWithSpeeds;
import frc.robot.commands.IntakeWithSpeeds;
import frc.robot.commands.LaunchFromPose;
import frc.robot.commands.SetLaunchParameters;
import frc.robot.commands.IntakeToPose;

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

    /* Path follower */
     private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
         autoChooser = AutoBuilder.buildAutoChooser("New Auto");
         SmartDashboard.putData("Auto Mode", autoChooser);


        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // breaking these out into separate functions for readability
        configureDriverBindings();
        // configureOperatorBindings();
        // configureTechBindings();
    }

    private void configureDriverBindings() {
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

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* Intake control */
        // deploy the intake, and intake game pieces
        driverController.leftTrigger(0.5)
            .onTrue(new IntakeToPose(intake, 3.9, 1))
            .whileTrue(new IntakeWithSpeeds(intake, indexer, 0.6, 0.3));

        // retract intake
        driverController.leftBumper().onTrue(new IntakeToPose(intake, 0, 0));
        
        /* launcher control */
        // hold trigger all the way for 0.5 seconds to launch stuff
        // advanced control for later
        // driverController.leftTrigger(1).debounce(0.5)
        //     .whileTrue(new LaunchFromPose(drivetrain, launcher, indexer, 0.6, 0.3));

        driverController.a().onTrue(drivetrain.applyRequest(() -> brake).andThen(new SetLaunchParameters(launcher, 3, 0.5)));
        driverController.b().whileTrue(new FeedWithSpeeds(indexer, 0.6, 0.3));
    }

    public Command getAutonomousCommand() {
       /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    //   return new InstantCommand();
    }
}

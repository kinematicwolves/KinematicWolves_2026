// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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
    private final SwerveRequest.FieldCentric fieldCentriceDrive = new SwerveRequest.FieldCentric()
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


    public RobotContainer() {

        configureBindings();
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
                fieldCentriceDrive
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

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Intake control
        driverController.leftTrigger(0.5)
            .onTrue(// deploy intake
                new InstantCommand(() -> intake.setPosition(3.9))
            )
            .whileTrue(// run intake
                new InstantCommand(() -> intake.setRollerSpeed(0.6)).andThen(
                )
            )
            .onFalse(// turn off intake
                new InstantCommand(() -> intake.setRollerSpeed(0.0)).andThen(
                new InstantCommand(() -> indexer.setRollerSpeed(0.3))
            )
        );

        driverController.leftBumper().onTrue(new InstantCommand(() -> intake.setPosition(0)));
        
        // launcher control
        // tap trigger past 50% to turn on launcher
        driverController.rightTrigger(0.5).debounce(0.1)
        .onTrue(// turn on launcher, hood to angle
            new InstantCommand(() -> launcher.setHoodPosition(3)).andThen(
            new InstantCommand(() -> launcher.setFlywheelPercent(0.6))
            )
        );

        // hold trigger all the way for 0.5 seconds to launch stuff
        driverController.leftTrigger(1).debounce(0.5)
            .whileTrue(// turn on feeder
                new InstantCommand(() -> indexer.setKickerspeed(-0.3)).andThen(
                new InstantCommand(() -> indexer.setRollerSpeed(0.6))
                )
            )
            .onFalse(// turn off feeder
                new InstantCommand(() -> indexer.setKickerspeed(0)).andThen(
                new InstantCommand(() -> indexer.setRollerSpeed(0))
                )
            );

        // right bumber turns off launcher
        driverController.rightBumper()
            .onTrue(// turn off launcher, move hood back
                new InstantCommand(() -> launcher.setHoodPosition(0)).andThen(
                new InstantCommand(() -> launcher.setFlywheelPercent(0))
                )
            );
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}

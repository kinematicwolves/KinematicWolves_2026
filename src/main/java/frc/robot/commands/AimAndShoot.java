package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LauncherProfile;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAndShoot {

    /**
     * Allows the driver to dodge while the robot automatically aims and spools.
     */
    public static Command teleopAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer, 
        DoubleSupplier vX, DoubleSupplier vY
    ) {
        return Commands.parallel(
            // 1. SWERVE: Translate normally, but hijack rotation for Odometry Aiming
            swerve.applySlowDrive(vX, vY, vision::getOdometryAimRate),

            // 2. LAUNCHER: Spool based on real-time Odometry distance
            launcher.continuousAimCommand(vision::getOdometryDistanceMeters),

            // 3. INDEXER: Wait for Launcher & Vision, then fire!
            Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned())
                    .andThen(indexer.feedShooterCommand())
        ).withName("TeleopAimAndShoot");
    }

    /**
     * AUTO: "Shoot Anywhere"
     * Stops the robot, aims, spools, fires, and safely ends the command so PathPlanner can continue.
     */
    public static Command autoAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer
    ) {
        // The Firing Sequence (This is our "Deadline")
        Command fireSequence = Commands.sequence(
            Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
            indexer.feedShooterCommand().withTimeout(LauncherProfile.kAutoShootTimerSec) // Adjust timeout during tuning
        );

        // Background Tasks (Aiming and Spooling via Odometry)
        return Commands.deadline(
            fireSequence, 
            swerve.applySlowDrive(() -> 0.0, () -> 0.0, vision::getOdometryAimRate), 
            launcher.continuousAimCommand(vision::getOdometryDistanceMeters)        
        )
        // Safety Stop: Ensure everything shuts off before PathPlanner takes over
        .andThen(Commands.runOnce(() -> {
            indexer.stop();
            launcher.stop();
        }))
        .withName("AutoAimAndShoot");
    }
}
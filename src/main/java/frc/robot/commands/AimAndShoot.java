package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * Command factory for complex aiming and shooting sequences.
 * This class coordinates Swerve, Vision, Launcher, Indexer, and Intake.
 */
public class AimAndShoot {

    // Toggle for "Machine Gun" mode (checks RPM constantly) vs "Send It" mode (checks once)
    public static final boolean USE_GATED_FEEDER = LauncherProfile.kShootBallsAtTargetSpeedOnly;

    /**
     * TELEOP: Automatic aiming while allowing driver translation (dodging).
     * @param vX/vY Driver joystick inputs for movement.
     */
    public static Command teleopAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer, Intake intake, 
        DoubleSupplier vX, DoubleSupplier vY
    ) {
        // Base action: Run indexer and intake rollers together to move ball into flywheels
        Command feedAction = indexer.feedShooterCommand()
            .alongWith(intake.runRollersCommand(IntakeProfile.kRollerVoltage));

        // Build the firing logic based on the USE_GATED_FEEDER flag
        Command fireSequence = USE_GATED_FEEDER 
            ? Commands.sequence(
                // GATED: Wait for speed/align -> Feed -> Pause if speed drops -> Repeat
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction.onlyWhile(() -> launcher.isReadyToFire() && vision.isOdometryAligned())
              ).repeatedly()
            : Commands.sequence(
                // UNGATED: Wait for speed/align once -> Feed continuously
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction 
              );

        return Commands.parallel(
            // 1. Move the robot (Hijack rotation for auto-aiming)
            swerve.applySlowDrive(vX, vY, vision::getOdometryAimRate),

            // 2. Adjust flywheels and hood angle based on live distance
            launcher.continuousAimCommand(vision::getOdometryDistanceMeters),

            // 3. Run the selected firing sequence
            fireSequence
        ).withName("TeleopAimAndShoot");
    }

    /**
     * AUTO: Complete "Stop, Aim, and Fire" sequence for PathPlanner.
     * Command ends after kAutoShootTimerSec so the next auto path can start.
     */
    public static Command autoAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer, Intake intake
    ) {
        Command feedAction = indexer.feedShooterCommand()
            .alongWith(intake.runRollersCommand(IntakeProfile.kRollerVoltage));

        // Build the auto-specific firing sequence with a hard timeout
        Command fireSequence = USE_GATED_FEEDER
            ? Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction.onlyWhile(() -> launcher.isReadyToFire() && vision.isOdometryAligned())
              ).repeatedly().withTimeout(LauncherProfile.kAutoShootTimerSec)
            : Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction 
              ).withTimeout(LauncherProfile.kAutoShootTimerSec);

        // DEADLINE: The entire command ends as soon as the fireSequence (timeout) is finished
        return Commands.deadline(
            fireSequence, 
            swerve.applySlowDrive(() -> 0.0, () -> 0.0, vision::getOdometryAimRate), // Hold still while aiming
            launcher.continuousAimCommand(vision::getOdometryDistanceMeters)        // Keep spooled
        )
        // Clean up: Ensure motors stop so we don't carry momentum into the next path
        .andThen(Commands.runOnce(() -> {
            indexer.stop();
            launcher.stop();
        }))
        .withName("AutoAimAndShoot");
    }
}
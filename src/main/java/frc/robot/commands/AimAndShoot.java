package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAndShoot {

    // --- TOGGLE THIS TO FALSE TO DISABLE SHOOTER GATE LOGIC ---
    public static final boolean USE_GATED_FEEDER = LauncherProfile.kShootBallsAtTargetSpeedOnly;

    /**
     * Allows the driver to dodge while the robot automatically aims and spools.
     */
    public static Command teleopAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer, Intake intake, XboxController driverController,
        DoubleSupplier vX, DoubleSupplier vY
    ) {
        // Base feed action (used in both gated and ungated)
        Command feedAction = indexer.feedShooterCommand()
            .alongWith(intake.runRollersCommand(IntakeProfile.kRollerVoltage));

        // Dynamically build the firing sequence based on our feature flag
        Command fireSequence = USE_GATED_FEEDER 
            ? Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && (vision.isOdometryAligned() || driverController.getPOV() == 270)),
                feedAction.onlyWhile(() -> launcher.isReadyToFire() && (vision.isOdometryAligned() || driverController.getPOV() == 270))
              ).repeatedly()
            : Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && (vision.isOdometryAligned() || driverController.getPOV() == 270)),
                feedAction // Ungated: Just waits once, then runs continuously
              );

        return Commands.parallel(
            // 1. SWERVE: Translate normally, but hijack rotation for Odometry Aiming
            swerve.applySlowDrive(vX, vY, vision::getOdometryAimRate),

            // 2. LAUNCHER: Spool based on real-time Odometry distance
            launcher.continuousAimCommand(vision::getOdometryDistanceMeters),

            // 3. THE FEEDER: Runs whichever sequence we built above
            fireSequence
        ).withName("TeleopAimAndShoot");
    }

    /**
     * AUTO: "Shoot Anywhere"
     * Stops the robot, aims, spools, fires, and safely ends the command so PathPlanner can continue.
     */
    public static Command autoAimAndShoot(
        Swerve swerve, Vision vision, Launcher launcher, Indexer indexer, Intake intake
    ) {
        // Base feed action
        Command feedAction = indexer.feedShooterCommand()
            .alongWith(intake.runRollersCommand(IntakeProfile.kRollerVoltage));

        // Dynamically build the auto firing sequence
        Command fireSequence = USE_GATED_FEEDER
            ? Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction.onlyWhile(() -> launcher.isReadyToFire() && vision.isOdometryAligned())
              ).repeatedly().withTimeout(LauncherProfile.kAutoShootTimerSec)
            : Commands.sequence(
                Commands.waitUntil(() -> launcher.isReadyToFire() && vision.isOdometryAligned()),
                feedAction 
              ).withTimeout(LauncherProfile.kAutoShootTimerSec);

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
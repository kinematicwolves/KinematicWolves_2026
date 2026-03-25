package frc.robot.commands.autoCommands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Swerve;

public class TowerTrajectory {

    /**
     * Generates a live PathPlanner trajectory to the correct alliance's climbing tower.
     * Evaluates the alliance color at the exact moment the command is scheduled.
     */
    public static Command autoClimbCommand(Swerve swerve) {
        return Commands.defer(() -> {
            // Check alliance AT THE TIME THE BUTTON IS PRESSED. Default to Blue if unknown.
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            
            // 2026 Tower Coordinates
            Pose2d redTowerPose = new Pose2d(15.902, 4.059, Rotation2d.fromDegrees(180));
            Pose2d blueTowerPose = new Pose2d(1.648, 4.059, Rotation2d.fromDegrees(0));
            
            // Select target based on alliance
            Pose2d targetPose = isRed ? redTowerPose : blueTowerPose;

            // Define the safe driving speeds for the auto-align
            PathConstraints constraints = new PathConstraints(
                1.2, // Max Velocity (m/s)
                0.6, // Max Acceleration (m/s^2)
                Units.degreesToRadians(360), // Max Angular Velocity (rad/s)
                Units.degreesToRadians(540)  // Max Angular Acceleration (rad/s^2)
            );

            // Return the PathPlanner generated command
            return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
            
        }, Set.of(swerve)).withName("AutoClimbPathfind");
    }
}
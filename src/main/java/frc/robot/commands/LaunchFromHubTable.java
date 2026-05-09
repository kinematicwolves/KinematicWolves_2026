// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaunchFromHubTable extends Command {
    /** Creates a new SetLaunchParametersFromPose. */
    private final Vision   visionSubsystem;
    private final Launcher launcherSubsystem;
    private final Indexer  indexerSubsystem;

    public LaunchFromHubTable(Vision visionSubsystem, Launcher launcherSubsystem, Indexer indexerSubsystem) {
        this.visionSubsystem   = visionSubsystem;
        this.launcherSubsystem = launcherSubsystem;
        this.indexerSubsystem  = indexerSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(launcherSubsystem, indexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // get the current distance from the robot to the goal from the vision subsystem
        double dist2goal = visionSubsystem.dist2pose(this.visionSubsystem.getAllianceGoal());
        Rotation2d rotation2gaol = visionSubsystem.rotation2Pose(this.visionSubsystem.getAllianceGoal());

        // pass the pose to the launcher
        this.launcherSubsystem.setParamsFromTable(dist2goal, LauncherProfile.hubShotTable);

        // if the launcher is good, launch
        if (this.launcherSubsystem.flywheelAtSpeed() && this.launcherSubsystem.hoodIsAtSetpoint() && Math.abs(rotation2gaol.getDegrees()) < 2) {
            this.indexerSubsystem.setRollerPercent(IndexerProfile.indexPercent);
            this.indexerSubsystem.setKickerPercent(IndexerProfile.feedPercent);
        }
        // otherwise, don't launch
        else {
            this.indexerSubsystem.turnOff();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.launcherSubsystem.turnOff();
        this.indexerSubsystem.turnOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

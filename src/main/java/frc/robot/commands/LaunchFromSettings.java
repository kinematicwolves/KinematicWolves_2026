// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IndexerProfile;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaunchFromSettings extends Command {
    /** Creates a new SetLaunchParametersFromPose. */
    private final Launcher launcherSubsystem;
    private final Indexer indexerSubsystem;
    private final RobotContainer robotContainer;
    private final XboxController driverController;

    /**
     * This is for launching with settings set by the operator, with driver confirmation (basically for use in teleop)
     * @param launcherSubsystem
     * @param indexerSubsystem
     * @param robotContainer
     * @param driverController
     */
    public LaunchFromSettings(Launcher launcherSubsystem, Indexer indexerSubsystem, RobotContainer robotContainer, XboxController driverController) {
        this.launcherSubsystem = launcherSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.robotContainer = robotContainer;
        this.driverController = driverController;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(launcherSubsystem, indexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // set the launcher from the current robot container settigns
        this.launcherSubsystem.setFlywheelSpeed(this.robotContainer.getLauncherSpeed());
        this.launcherSubsystem.setHoodPosition(this.robotContainer.getLauncherAngle());

        // if the launcher is good, launch
        if (this.launcherSubsystem.flywheelAtSpeed() && this.driverController.getRightBumperButton()) {
            this.indexerSubsystem.setRollerPercent(IndexerProfile.indexPercent);
            this.indexerSubsystem.setKickerPercent(IndexerProfile.indexPercent);
        }
        // otherwise, don't launch
        else {
            this.indexerSubsystem.setRollerPercent(0);
            this.indexerSubsystem.setKickerPercent(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.launcherSubsystem.setHoodPosition(0);
        this.launcherSubsystem.setFlywheelPercent(0);
        this.indexerSubsystem.setKickerPercent(0);
        this.indexerSubsystem.setRollerPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

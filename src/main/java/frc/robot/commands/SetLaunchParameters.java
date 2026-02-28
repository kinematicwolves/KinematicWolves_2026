// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetLaunchParameters extends Command {
    /** Creates a new SetLaunchParameters. */
    private final Launcher launcherSubsystem;
    private final double hoodPose;
    private final double launcherPercent;

    public SetLaunchParameters(Launcher launcherSubsystem, double hoodPose, double launcherPercent) {
        this.launcherSubsystem = launcherSubsystem;
        this.hoodPose = hoodPose;
        this.launcherPercent = launcherPercent;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(launcherSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        launcherSubsystem.setFlywheelPercent(this.launcherPercent);
        launcherSubsystem.setHoodPosition(this.hoodPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.launcherSubsystem.hoodIsAtSetpoint(); //TODO: add rpms condition
    }
}

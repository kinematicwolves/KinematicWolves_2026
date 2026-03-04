// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaunchwithParams extends Command {
    /** Creates a new LaunchwithParams. */
    private Launcher launcher;
    private double launcherPercent;
    private double hoodPose;
    private RobotContainer robotContainer;

    public LaunchwithParams(Launcher launcherSubsystem, RobotContainer robotContainer, double launchPercent, double hoodPose) {
        this.launcher = launcherSubsystem;
        this.launcherPercent = launchPercent;
        this.hoodPose = hoodPose; 
        this.robotContainer = robotContainer;
    
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(launcherSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // this.launcher.setFlywheelPercent(this.launcherPercent);
        // this.launcher.setHoodPosition(this.hoodPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.launcher.setFlywheelPercent(this.robotContainer.getLauncherPercent());
        this.launcher.setHoodPosition(this.robotContainer.getLauncherAngle());
        System.out.print(this.robotContainer.getLauncherPercent());
        System.out.print("    ");
        System.out.println(this.robotContainer.getLauncherAngle());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

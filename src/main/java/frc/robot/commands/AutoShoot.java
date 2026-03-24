// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;

public class AutoShoot extends Command {
    private final Launcher launcher;
    private final Indexer indexer;
    private final CommandSwerveDrivetrain drivetrain;
    
    private final Timer shootTimer = new Timer();
    private boolean isShooting = false;

    /**
     * A unified shooting command that calculates shot parameters dynamically based on the robot's pose.
     * Works perfectly for both Autonomous and TeleOp.
     */
    public AutoShoot(Launcher launcher, Indexer indexer, CommandSwerveDrivetrain drivetrain) {
        this.launcher = launcher;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        
        // We require the Launcher and Indexer so no other commands can hijack them.
        // We DO NOT require the Drivetrain, allowing the robot to drive/aim while spooling!
        addRequirements(launcher, indexer);
    }

    @Override
    public void initialize() {
        isShooting = false;
        shootTimer.stop();
        shootTimer.reset();
    }

    @Override
    public void execute() {
        // 1. Continuously update the target RPM and Hood Angle based on where the robot is RIGHT NOW.
        // Doing this in execute() means we can shoot on the move!
        launcher.setHoodAndSpeedFromPose(drivetrain.getPose());

        // 2. Check if our hardware is ready to fire
        if (launcher.flywheelAtSpeed() && launcher.hoodIsAtSetpoint()) {
            
            // If this is the exact moment we reached speed, start the timer
            if (!isShooting) {
                isShooting = true;
                shootTimer.start();
            }
            
            indexer.setKickerPercent(1.0);
            indexer.setRollerPercent(1.0);
            
        } else {
            // If the flywheels dip below target speed (e.g., if a note gets stuck, 
            // or if we are still spooling), keep the indexer stopped.
            indexer.setKickerPercent(0.0);
            indexer.setRollerPercent(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // The command automatically finishes 0.5 seconds AFTER the flywheels reached speed 
        // and the indexer started pushing the note. (Tune this time if needed!)
        return isShooting && shootTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up everything when the shot is done or if the command is cancelled
        launcher.turnFlywheelOff();
        indexer.setKickerPercent(0.0);
        indexer.setRollerPercent(0.0);
    }
}
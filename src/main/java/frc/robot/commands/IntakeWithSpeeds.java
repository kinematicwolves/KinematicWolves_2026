// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeWithSpeeds extends Command {
    /** Creates a new IntakeWithSpeeds. */
    // 1) setup all internal references, variables, etc
    private final Intake intakeSubsystem;
    private final Indexer indexerSubsystem;
    private final double intakePercent;
    private final double indexerPercent;

    public IntakeWithSpeeds(Intake intakeSubsystem, Indexer indexerSubystem, double intakePercent, double indexerPercent) {
        // 2) pass all values to the internal variables
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubystem;
        this.intakePercent = intakePercent;
        this.indexerPercent = indexerPercent;

        // Use addRequirements() here to declare subsystem dependencies.
        // 3) Tell the command to take control of the subsystem
        addRequirements(intakeSubsystem, indexerSubystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.intakeSubsystem.setRollerPercent(this.intakePercent);
        this.indexerSubsystem.setRollerPercent(this.indexerPercent);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.setRollerPercent(0);
        this.indexerSubsystem.setRollerPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

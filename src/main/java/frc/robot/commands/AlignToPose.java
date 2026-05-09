// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPose extends Command {
    /** Creates a new AlignToPose. */
    private CommandSwerveDrivetrain drivetrain;
    private Vision visionSubsystem;
    private Pose2d alignmentPose;
    private DoubleSupplier velocityX;
    private DoubleSupplier velocityY;

    private PIDController rotationaLPidController;
    private SwerveRequest.RobotCentric robotCentricDrive;

    public AlignToPose(CommandSwerveDrivetrain drivetrain, Vision vision, Pose2d alignmentPose, DoubleSupplier x, DoubleSupplier y, SwerveRequest.RobotCentric robotCentricDrive) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = vision;
        this.alignmentPose = alignmentPose;
        this.velocityX = x;
        this.velocityY = y;
        this.robotCentricDrive = robotCentricDrive;

        this.rotationaLPidController = new PIDController(0.01, 0, 0);
        this.rotationaLPidController.setSetpoint(0); // degrees
        this.rotationaLPidController.setTolerance(2); // degrees
        this.rotationaLPidController.enableContinuousInput(-180, 180);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // compute the rotation between the robot and the pose we want to point at
        Rotation2d rotation2pose = this.visionSubsystem.rotation2Pose(alignmentPose);

        // compute a new rotational rate for the swerve drive
        double rotationalRate = this.rotationaLPidController.calculate(rotation2pose.getDegrees());

        // and apply the drive request to the swerve drive
        this.drivetrain.setControl(
            this.robotCentricDrive
                .withVelocityX(this.velocityX.getAsDouble())
                .withVelocityY(this.velocityY.getAsDouble())
                .withRotationalRate(rotationalRate)
        );

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.rotationaLPidController.atSetpoint();
    }
}

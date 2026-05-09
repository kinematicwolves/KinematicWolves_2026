// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherProfile;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelight = "limelight";
    public final Field2d field = new Field2d();

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (true) {
            var driveState = this.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                this.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }

        // update the robot pose on the field display
        this.field.setRobotPose(this.drivetrain.getState().Pose);

        // SmartDashbord ouputs
        SmartDashboard.putNumber("Dist2RedGoal",  this.robot2Pose(LauncherProfile.redHub).getTranslation().getNorm());
        SmartDashboard.putNumber("Dist2BlueGoal", this.robot2Pose(LauncherProfile.blueHub).getTranslation().getNorm());
        SmartDashboard.putData(this.field);
    }

    public Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /* Generic Transform functions */
    /**
     * Computes the 2d transform between the robot and the other pose.
     * @param otherPose
     * @return the 2d transform.
     */
    public Transform2d robot2Pose(Pose2d otherPose) {
        // First, get the current current robot pose from the drive train
        Pose2d robotPose = drivetrain.getState().Pose;

        // Then, compute the transform between the robot and the goal
        return otherPose.minus(robotPose);
    }

    /**
     * Compputes the distance between the robots pose and another position.
     * @param otherPose
     * @return the 2d distance between the robot and the other pose.
     */
    public double dist2pose(Pose2d otherPose) {
        // First, get the robot2pose transform
        Transform2d robot2pose = this.robot2Pose(otherPose);

        // Then, compute the magnitude of the transform (fancy talk for the distance)
        return robot2pose.getTranslation().getNorm();
    }

    /**
     * Computes the rotation between the robot orientation and another pose. Think of this as how far the robot must turn to point at that pose.
     * @param otherPose
     * @return the rotation between the robot orientation and the other pose.
     */
    public Rotation2d rotation2Pose(Pose2d otherPose) {
        // First, get the robot2pose transform
        Transform2d robot2pose = this.robot2Pose(otherPose);

        // Next, get the rotation of the robot
        Rotation2d targetAngle = new Rotation2d(robot2pose.getX(), robot2pose.getY());

        // We don't want the difference between the robot heading and the goal.
        // Instead, we want the difference between the robot heading and the vector (line) of the robot and the goal
        return targetAngle.minus(drivetrain.getState().Pose.getRotation());
    }
}

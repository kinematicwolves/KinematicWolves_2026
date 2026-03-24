// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDProfile;
import frc.robot.Constants.LauncherProfile;
import frc.robot.utils.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.lighting.setSegmentColor(LEDProfile.CANdleLeds, LEDProfile.green);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (true) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            // 1. Send the ultra-accurate gyro heading to Limelight for MegaTag2
            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            
            // 2. USE MegaTag2. This completely eliminates 3D ambiguity flipping.
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            
            if (llMeasurement != null && llMeasurement.tagCount > 0) {
                
                // Reject updates if the robot is spinning too fast (causes motion blur)
                if (Math.abs(omegaRps) > 2.0) {
                    return; // Skip this loop
                }

                // Reject updates if it's only 1 tag and we are super far away (e.g., > 4 meters)
                double avgDist = llMeasurement.avgTagDist;
                if (llMeasurement.tagCount == 1 && avgDist > 4.0) {
                    return; // Too risky to trust
                }

                // 3. Dynamic Standard Deviations: Trust = (Distance^2) / TagCount
                // The further away, the larger the deviation (less trust). More tags = more trust.
                double xyStdDev = 0.01 * Math.pow(avgDist, 2) / llMeasurement.tagCount;
                
                // Add the measurement. We use 9999999 for the rotation standard deviation 
                // because MegaTag2 relies on the gyro anyway. We don't want vision changing our heading.
                m_robotContainer.drivetrain.addVisionMeasurement(
                    llMeasurement.pose, 
                    llMeasurement.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999) 
                );
            }
        }
        SmartDashboard.putNumber("OpLauncherSpeed", m_robotContainer.getLauncherSpeed());
        SmartDashboard.putNumber("OpLauncherAngle", m_robotContainer.getLauncherAngle());
        SmartDashboard.putNumber("Dist2Redgoal", LauncherProfile.redHub.getTranslation().getDistance(m_robotContainer.drivetrain.getPose().getTranslation()));
        SmartDashboard.putNumber("Dist2Bluegoal", LauncherProfile.blueHub.getTranslation().getDistance(m_robotContainer.drivetrain.getPose().getTranslation()));
    }

    @Override
    public void disabledInit() {
        m_robotContainer.lighting.setSegmentColor(LEDProfile.CANdleLeds, LEDProfile.green);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.lighting.setSegmentRainbowAnimation(LEDProfile.CANdleLeds);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
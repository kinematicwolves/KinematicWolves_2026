// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {
        public final class LEDProfile {
        public static final RGBWColor pink   = new RGBWColor(255,  31, 156);
        public static final RGBWColor red    = new RGBWColor(255,   0,   0);
        public static final RGBWColor green  = new RGBWColor(  0, 217,   0);
        public static final RGBWColor blue   = new RGBWColor(  0,   0, 255);
        public static final RGBWColor white  = new RGBWColor(255, 255, 255);
        public static final RGBWColor orange = new RGBWColor(255, 128,   0);
        public static final RGBWColor off    = new RGBWColor(  0,   0,   0);
        public static final double BrightnessScalar = 0.5;
        
        public static final CANdleSegment CANdleLeds = new CANdleSegment( 0,  7, 1);
    }
    public static class LauncherProfile {
        public static final int launcherMotor1CanID = 41;
        public static final int launcherMotor2CanID = 42;
        public static final int hoodMotorCanID      = 44;

        public static final double launcherTolerance = 1; // rotations/s

        public static final Pose2d blueHub = new Pose2d( 4.642, 4.075, new Rotation2d());
        public static final Pose2d redHub  = new Pose2d(11.981, 4.075, new Rotation2d());

        public static final double idealLaunchDist = 2.5; // meters
    }

    public static class IntakeProfile {
        public static final int intakeMotorACanID = 31;
        public static final int intakeMotorBCanID = 32;
        public static final int rollerMotorCanID  = 33;
        
        public static final int gentleSlot     = 0;
        public static final int aggressiveSlot = 1;

        public static final double deployedPose   =  0.0; // rotations
        public static final double retractedPose  = -4.0; // rotations
        public static final double poseTolerance  =  0.2; // rotations
        public static final double intakePercent  =  1.0; // percent
        public static final double retractPrecent = -0.5; // percent
    }

    public static final class IndexerProfile {
        public static final int kickerMotorCanID = 43;
        public static final int rollerMotorCanID = 34;

        public static final double indexPercent = 0.6; // percent
        public static final double feedPercent  = 1.0; // percent
    }

    public class ElevatorProfile {
        public static final int motor1ID = 21; // TODO: confirm CanID
        public static final int motor2ID = 22; // TODO: confirm CanID
    }

}

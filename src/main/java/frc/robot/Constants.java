// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class LEDProfile {
        public static final RGBWColor pink   = new RGBWColor(255,  31, 156);
        public static final RGBWColor red    = new RGBWColor(255,   0,   0);
        public static final RGBWColor green  = new RGBWColor(  0, 217,   0);
        public static final RGBWColor blue   = new RGBWColor(  0,   0, 255);
        public static final RGBWColor white  = new RGBWColor(255, 255, 255);
        public static final RGBWColor orange = new RGBWColor(255, 128,   0);
        public static final RGBWColor off    = new RGBWColor(  0,   0,   0);
        public static final double BrightnessScalar = 0.5;
        
        public static Map<String, CANdleSegment> CANdleSegments = Map.of(
            "candle", new CANdleSegment( 0,  7, 1)
        );
    }

    public final class LauncherProfile {
        public static final Pose2d blueHub = new Pose2d( 4.642, 4.075, null); // in wpiblue coordinates, from pathplanner gui
        public static final Pose2d redHub  = new Pose2d(11.981, 4.075, null); // in wpiblue coordinates, from pathplanner gui
        public static final int launcherMotor1ID = 41;
        public static final int launcherMotor2ID = 42;
        public static final int hoodMotorID      = 44;

        public static final double speedTolerance = 1; // rotation/s
        public static final double hoodTolerance  = 0.1; // rotations

    }

    public final class IntakeProfile {
        public static final int intakeDeployMotor1ID = 31;
        public static final int intakeDeployMotor2ID = 32;
        public static final int intakeRollerMotorID  = 33;
        public static final double deployTolerance = 0.1;

        public static final double deployPose = 3.9;
        public static final double zeroPose   = 0.0;
    }

    public final class IndexerProfile {
        public static final int kickerMotorID = 43;
        public static final int rollerMotorID = 34;
    }
}

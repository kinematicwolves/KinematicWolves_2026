// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.RGBWColor;

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
}

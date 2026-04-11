package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public final class Constants {

    public static final class SwerveProfile {
    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); 
    public static final double kMaxAngularRate = 0.55 * Math.PI;
    public static final double kSlowTranslationScalar = 0.2; // Shoot while moving speed

    // PathPlanner PID Constants
    public static final PIDConstants kTranslationPID = new PIDConstants(2.5, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(2, 0.0, 0.0);
    }

    public static final class VisionProfile {
        public static final String kLimelightName = "limelight";
        
        // PID for Auto-Aiming (Rotation)
        public static final double kP_Align = 0.03; 
        public static final double kI_Align = 0.0;
        public static final double kD_Align = 0.001;
        
        // Tolerance for "Locked On" status
        public static final double kAlignToleranceDegrees = 2.5;

        // "Home Shot" Fallback Constants (Fender shot)
        public static final double kFallbackRPM = 3500;
        public static final double kFallbackHoodPosition = 15.0; 
    }

    public static final class FieldConstants {
        // Target height for distance calculations
        public static final double kTargetCenterHeightMeters = 2.64; // ~104 inches converted to meters
        public static final double kLimelightMountHeightMeters = 0.62; // ~24.5 inches converted to meters
        public static final double kLimelightMountAngleDegrees = 35.0;
        public static final Translation2d kBlueHub = new Translation2d(4.642, 4.075);
        public static final Translation2d kRedHub = new Translation2d(11.981, 4.075);
    }

    public static final class IntakeProfile {
        public static final int kPivotMasterID = 31; 
        public static final int kPivotFollowerID = 32; 
        public static final int kRollerID = 33; 

        // Pivot Positions (Motor Rotations, NOT ticks)
        public static final double kPivotUpPosition = 0;
        public static final double kPivotDownPosition = 4.1; //
        public static final double kPivotTolerance = 0.5;

        // Motor Speeds
        public static final double kRollerVoltage = 11.0; 
        public static final double kExhaustVoltage = -11; // TODO: Set voltage if needed

        // Current Limits
        public static final double kPivotCurrentLimit = 20; // Amps
        public static final int kRollerCurrentLimit = 30; // Amps

        // PID & Motion Magic for Kraken Pivot
        public static final double kPivotP = 0.9; //TODO: Tune
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.01;
        public static final double kPivotMaxVelocity = 5; // Rotations per second
        public static final double kPivotMaxAcceleration = 10; // Rotations per second squared
    }

    public static final class LauncherProfile {
        public static final int kFlywheelLeftID = 41; 
        public static final int kFlywheelRightID = 42;
        public static final int kHoodID = 44;

        // Thresholds
        public static final double kRPSTolerance = 2.5; //TODO: Tune
        public static final double kHoodTolerance = 0.1; // TODO: Tune (Rotations)

        // Hood Config
        public static final double kHoodP = 2; // TODO: Tune
        public static final double kHoodMinPosition = 0.0;
        public static final double kHoodMaxPosition = 5; // Total travel rotations

        // Flywheel PID
        public static final double kFlywheelP = 0.45; //TODO: Tune both p and feedfoward
        public static final double kFlywheelV = 0.13; // Feedforward is key for flywheels

        // --- INTERPOLATING TABLE DATA ---
        // Key: Distance (meters), Value: {Flywheel RPS, Hood Rotations}
        public static final double[][] kShootingData = { //TODO: Tune
            {2, 44, 0},   // Close (Front of hub)
            {2.8, 52, 0},  // Mid Range
            {3.7, 67, 0},  // Long Range
           // {4.1, 73, 0}   // Maximum Distance
        };

        // Timer for auto shoot command for pathplanner
        public static final double kAutoShootTimerSec = 7.5;

        // If true, the indexer will only feed balls when the flywheel is at the target speed. If false, it will feed continuously.
        // REMEMBER: If you disable this, make sure to also disable in the closeShotCommand in the Launcher subsystem.
        public static final boolean kShootBallsAtTargetSpeedOnly = true;
    }

    public static final class IndexerProfile {
        public static final int kHopperID = 34; // 775 / Talon SRX
        public static final int kKickerID = 43; // NEO / Spark Max

        // Forward/Feed Voltages
        public static final double kHopperVoltage = 7.2;
        public static final double kKickerVoltage = 11.0;
        
        // Reverse/Exhaust Voltages
        public static final double kReverseVoltage = -8.0;

        // Current Limits (Prevents brownouts when rapidly firing)
        public static final int kHopperCurrentLimit = 15; // Amps
        public static final int kKickerCurrentLimit = 30; // Amps
    }

    public static final class ClimberProfile {
        public static final int kClimberLeftID = 23; 
        public static final int kClimberRightID = 24;

        // TODO: Config Positions (USES ROTATIONS, NOT TICKS)
        public static final double kMaxHeight = 50.0;
        public static final double kHomePosition = 0;
        public static final double kTolerance = 0.5; 
        
        // PID & Motion Magic
        public static final double kClimberP = 2.0; //TODO: Tune
        public static final double kClimberV = 0.12; 
        
        public static final double kMaxVelocity = 60.0; // Rotations per second
        public static final double kMaxAcceleration = 120.0; // Rotations per sec^2
    }

    public static final class LightingProfile {
        public static final int kCandleID = 5;
        
        /* LED Strip Sizing */
        public static final int kNumLeds = 150;   // Number of external LEDs
        public static final int kStartIndex = 8; // Skips the 8 CANdle LEDs
        
        /* Disabled Animation Settings */
        public static final double kRainbowBrightness = 0.1; 
        public static final int kRainbowFrameRate = 10; // Animation speed

        /* Predictive Hub Timings (Seconds) */
        public static final double kHubActivateWarningSec = 5.0;   // Show green 5s before active
        public static final double kHubDeactivateWarningSec = 3.0; // Revert to alliance 3s before hub becomes inactive
    }
}
package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final class SwerveProfile {
        /* Driving Speeds (Meters per Second) */
        public static final double kMaxSpeed = 1.2; 
        public static final double kMaxAngularRate = 0.6 * Math.PI;
        public static final double kSlowTranslationScalar = 0.2; // Multiplier for Aim-and-Shoot mode

        /* PathPlanner PID: Tune these if the robot wobbles or overshoots during Auto */
        public static final PIDConstants kTranslationPID = new PIDConstants(2.5, 0.0, 0.0);
        public static final PIDConstants kRotationPID = new PIDConstants(2.0, 0.0, 0.0);
    }

    public static final class VisionProfile {
        public static final String kLimelightName = "limelight";
        
        /* Rotation PID: Controls how aggressively the robot snaps to face the target */
        public static final double kP_Align = 0.05; 
        public static final double kI_Align = 0.0;
        public static final double kD_Align = 0.001;
        
        /* If Tx is within this many degrees, we consider the robot "Locked On" */
        public static final double kAlignToleranceDegrees = 2.5;
    }

    public static final class FieldConstants {
        /* Physical heights used for trig-based distance calculations */
        public static final double kTargetCenterHeightMeters = 2.64; // Center of the goal
        public static final double kLimelightMountHeightMeters = 0.62; // Lens height from floor
        public static final double kLimelightMountAngleDegrees = 35.0; // Angle of camera tilt

        /* Hub coordinates (WPI Blue Origin) */
        public static final Translation2d kBlueHub = new Translation2d(4.642, 4.075);
        public static final Translation2d kRedHub = new Translation2d(11.981, 4.075);
    }

    public static final class IntakeProfile {
        public static final int kPivotMasterID = 31; 
        public static final int kPivotFollowerID = 32; 
        public static final int kRollerID = 33; 

        /* Pivot Positions (In Motor Rotations) */
        public static final double kPivotUpPosition = 0;
        public static final double kPivotDownPosition = 5.1; // Ground intake position
        public static final double kPivotTolerance = 0.5;

        /* Roller Speeds (0 to 12 Volts) */
        public static final double kRollerVoltage = 11; 
        public static final double kExhaustVoltage = -8.0; // Reverse to eject

        /* Safety Limits */
        public static final double kPivotCurrentLimit = 20; 
        public static final int kRollerCurrentLimit = 40; 

        /* Motion Magic: Controls the "feel" and smoothness of the intake arm */
        public static final double kPivotP = 0.8; 
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.01;
        public static final double kPivotMaxVelocity = 5; 
        public static final double kPivotMaxAcceleration = 10; 
    }

    public static final class LauncherProfile {
        public static final int kFlywheelLeftID = 41; 
        public static final int kFlywheelRightID = 42;
        public static final int kHoodID = 44;

        /* Tolerances: How close do we need to be to "Target" before we fire? */
        public static final double kRPSTolerance = 1.8; // Flywheel speed tolerance
        public static final double kHoodTolerance = 0.1; // Hood position tolerance

        /* Hood Tuning (SparkMax Position Control) */
        public static final double kHoodP = 2; 
        public static final double kHoodMinPosition = 0.0;
        public static final double kHoodMaxPosition = 5; 

        /* Flywheel Tuning (Kraken Velocity Control) */
        public static final double kFlywheelP = 0.45; 
        public static final double kFlywheelV = 0.13; // kV is the most important for consistent speed

        /* --- THE SHOT MAP --- */
        /* Format: { Distance(m), Flywheel RPS, Hood Rotations } */
        public static final double[][] kShootingData = { 
            {1.5, 49, 0},   // Front of hub
            {2.6, 54.5, 0},  // "Close to tower"
            {3.55, 66, 0},  // Trench shots
            {4.0, 81, 0}    // "Near" fuel source
        };

        /* How long the indexer runs during Auto before moving to next path */
        public static final double kAutoShootTimerSec = 3.0;

        /* GATED FEEDER TOGGLE: Set to false for "Non-stop shooting" mode (no RPM check) */
        public static final boolean kShootBallsAtTargetSpeedOnly = true; // REMEMBER to also disable in closeShotCommand (Launcher subsystem) if changing this!
    }

    public static final class IndexerProfile {
        public static final int kHopperID = 34; 
        public static final int kKickerID = 43; 

        /* Forward Voltages */
        public static final double kHopperVoltage = 7.2;
        public static final double kKickerVoltage = 11.0;
        
        /* Reverse Voltage (Clear jams) */
        public static final double kReverseVoltage = -8.0;

        /* Current Limits: Low hopper limit avoids crushing balls */
        public static final int kHopperCurrentLimit = 30; 
        public static final int kKickerCurrentLimit = 40; 
    }

    public static final class ClimberProfile {
        public static final int kClimberLeftID = 23; 
        public static final int kClimberRightID = 24;

        /* Height Limits (Rotations) */
        public static final double kMaxHeight = 50.0;
        public static final double kHomePosition = 0;
        public static final double kTolerance = 0.5; 
        
        /* Motion Magic Tuning */
        public static final double kClimberP = 2.0; 
        public static final double kClimberV = 0.12; 
        public static final double kMaxVelocity = 60.0; 
        public static final double kMaxAcceleration = 120.0; 
    }

    public static final class LightingProfile {
        public static final int kCandleID = 5;
        
        /* LED Strip Sizing */
        public static final int kNumLeds = 65;   // Number of external LEDs
        public static final int kStartIndex = 8; // Skips the 8 CANdle LEDs
        
        /* Disabled Animation Settings */
        public static final double kRainbowBrightness = 0.1; 
        public static final int kRainbowFrameRate = 10; // Animation speed

        /* Predictive Hub Timings (Seconds) */
        public static final double kHubActivateWarningSec = 5.0;   // Show green 5s before active
        public static final double kHubDeactivateWarningSec = 3.0; // Revert to alliance 3s before hub becomes inactive
    }
}
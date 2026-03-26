package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionProfile;
import frc.robot.Constants.FieldConstants;

public class Vision extends SubsystemBase {

    private final NetworkTable m_limelightTable;
    private final Swerve m_swerve;

    // Controllers for turning toward the target
    private final PIDController m_limelightAimController; // Raw Tx backup
    private final PIDController m_odometryAimController;  // Pose-based primary

    // Flag to ensure we only hard-reset gyro once on boot
    private boolean m_hasSeededOdometry = false;

    public Vision(Swerve swerve) {
        m_swerve = swerve;
        m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionProfile.kLimelightName);
        
        // Setup raw vision backup controller
        m_limelightAimController = new PIDController(
            VisionProfile.kP_Align, 
            VisionProfile.kI_Align, 
            VisionProfile.kD_Align
        );
        m_limelightAimController.setSetpoint(0.0);
        m_limelightAimController.setTolerance(VisionProfile.kAlignToleranceDegrees);

        // Setup odometry-based aiming (rotation)
        m_odometryAimController = new PIDController(0.05, 0.0, 0.1); 
        m_odometryAimController.enableContinuousInput(-Math.PI, Math.PI); // Logic for -180 to 180 wrap
        m_odometryAimController.setTolerance(Units.degreesToRadians(2.0));
    }

    /* ========================================================= */
    /* POSE-BASED SCORING (PRIMARY METHOD)                       */
    /* ========================================================= */

    /** Returns the XY coordinates of the goal based on alliance color */
    public Translation2d getTargetHub() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? FieldConstants.kRedHub : FieldConstants.kBlueHub;
    }

    /** Calculates 2D distance between robot and the target hub */
    public double getOdometryDistanceMeters() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        return robotPos.getDistance(getTargetHub());
    }

    /** Returns rotation speed to face the hub; adds 180 because shooter is on the back */
    public double getOdometryAimRate() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        Translation2d target = getTargetHub();
        
        // Vector math to find the angle to target
        Rotation2d targetHeading = target.minus(robotPos).getAngle();
        
        // Offset for back-mounted shooter
        targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));

        return m_odometryAimController.calculate(
            m_swerve.getPose().getRotation().getRadians(), 
            targetHeading.getRadians()
        );
    }

    /** Returns true if robot rotation is within tolerance of the target */
    public boolean isOdometryAligned() {
        return m_odometryAimController.atSetpoint();
    }

    /* ========================================================= */
    /* RAW LIMELIGHT DATA & FALLBACKS                            */
    /* ========================================================= */

    /** tv: returns true if Limelight sees ANY valid target */
    public boolean hasTarget() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /** tx: horizontal offset from crosshair to target */
    public double getTx() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    /** Trig-based distance: uses fixed camera height/angle vs target height */
    public double getDistanceMeters_Fallback() {
        if (!hasTarget()) return 0.0;
        double targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);
        double angleToGoalRadians = Math.toRadians(FieldConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical);
        double heightDifference = FieldConstants.kTargetCenterHeightMeters - FieldConstants.kLimelightMountHeightMeters;
        
        // d = h / tan(a)
        return heightDifference / Math.tan(angleToGoalRadians);
    }

    /** Uses PID to turn based on tx alone (ignores robot position) */
    public double getSteeringFeedback_Fallback() {
        if (!hasTarget()) return 0.0; 
        return -m_limelightAimController.calculate(getTx());
    }

    /* ========================================================= */
    /* MEGATAG ODOMETRY UPDATES                                  */
    /* ========================================================= */

    /** Pulls MegaTag pose from Limelight and injects it into Swerve PoseEstimator */
    private void updateOdometry() {
        if (!hasTarget()) return;

        // botpose_wpiblue is the 2026 standard for field-relative position
        double[] botpose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        
        // Limelight firmware update uses index >= 7 for MegaTag2 data
        if (botpose.length >= 7) {
            Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
            
            // Subtract camera processing time from current timestamp to align data history
            double tl = m_limelightTable.getEntry("tl").getDouble(0.0);
            double cl = m_limelightTable.getEntry("cl").getDouble(0.0);
            double latencySeconds = (tl + cl) / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latencySeconds;

            // First-time seed: Snap odometry to vision to fix gyro startup errors
            if (!m_hasSeededOdometry) {
                m_swerve.resetPose(visionPose);
                m_hasSeededOdometry = true;
                System.out.println("SUCCESS: Odometry Seeded from Limelight!");
            } else {
                // Regular update: Merge vision with wheel encoders
                m_swerve.addVisionMeasurement(visionPose, timestamp);
            }
        }
    }

    @Override
    public void periodic() {
        // Send alliance color to Limelight so it knows which tags to prioritize
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            double allianceValue = (alliance.get() == Alliance.Red) ? 0.0 : 1.0;
            m_limelightTable.getEntry("alliance").setDouble(allianceValue);
        }

        updateOdometry();

        // Debugging values
        SmartDashboard.putNumber("Shooting/Odometry Distance", getOdometryDistanceMeters());
        SmartDashboard.putBoolean("Shooting/Is Aligned", isOdometryAligned());
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
    }
}
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private final PIDController m_limelightAimController;
    private final PIDController m_odometryAimController;

    // Track if we have synced the gyro to the field yet
    private boolean m_hasSeededOdometry = false;

    public Vision(Swerve swerve) {
        m_swerve = swerve;
        m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionProfile.kLimelightName);
        
        m_limelightAimController = new PIDController(
            VisionProfile.kP_Align, 
            VisionProfile.kI_Align, 
            VisionProfile.kD_Align
        );
        m_limelightAimController.setSetpoint(0.0);
        m_limelightAimController.setTolerance(VisionProfile.kAlignToleranceDegrees);

        m_odometryAimController = new PIDController(5.0, 0.0, 0.1); 
        m_odometryAimController.enableContinuousInput(-Math.PI, Math.PI);
        m_odometryAimController.setTolerance(Units.degreesToRadians(2.0));
    }

    /* ========================================================= */
    /* POSE-BASED SCORING (PRIMARY METHOD)                       */
    /* ========================================================= */

    public Translation2d getTargetHub() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? FieldConstants.kRedHub : FieldConstants.kBlueHub;
    }

    public double getOdometryDistanceMeters() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        return robotPos.getDistance(getTargetHub());
    }

    public double getOdometryDownrangeMeters() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        return Math.abs(robotPos.getX() - getTargetHub().getX());
    }

    public double getOdometryAimRate() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        Translation2d target = getTargetHub();
        
        Rotation2d targetHeading = target.minus(robotPos).getAngle();
        targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));

        return m_odometryAimController.calculate(
            m_swerve.getPose().getRotation().getRadians(), 
            targetHeading.getRadians()
        );
    }

    public boolean isOdometryAligned() {
        return m_odometryAimController.atSetpoint();
    }

    /* ========================================================= */
    /* RAW LIMELIGHT DATA & FALLBACKS                            */
    /* ========================================================= */

    public boolean hasTarget() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getDistanceMeters_Fallback() {
        if (!hasTarget()) return 0.0;
        double targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);
        double angleToGoalRadians = Math.toRadians(FieldConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical);
        double heightDifference = FieldConstants.kTargetCenterHeightMeters - FieldConstants.kLimelightMountHeightMeters;
        return heightDifference / Math.tan(angleToGoalRadians);
    }

    public double getSteeringFeedback_Fallback() {
        if (!hasTarget()) return 0.0; 
        return -m_limelightAimController.calculate(getTx());
    }

    /* ========================================================= */
    /* MEGATAG ODOMETRY UPDATES                                  */
    /* ========================================================= */

    private void updateOdometry() {
        if (!hasTarget()) return;

        // Pull the array, defaulting to an empty array so we don't get null pointer crashes
        double[] botpose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        
        // FIX: Check for >= 7 so it works with the 11-element arrays in modern Limelight firmware!
        if (botpose.length >= 7) {
            Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
            
            // FIX: Pull exact latency directly from network tables (Pipeline + Capture latency)
            double tl = m_limelightTable.getEntry("tl").getDouble(0.0);
            double cl = m_limelightTable.getEntry("cl").getDouble(0.0);
            double latencySeconds = (tl + cl) / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latencySeconds;

            // Seed Odometry on first detection
            if (!m_hasSeededOdometry) {
                m_swerve.resetPose(visionPose);
                m_hasSeededOdometry = true;
                System.out.println("SUCCESS: Odometry Seeded from Limelight!");
            } else {
                // Now this actually runs!
                m_swerve.addVisionMeasurement(visionPose, timestamp);
            }
        }
    }

    @Override
    public void periodic() {
        // 'botpose_wpiblue' is permanently locked to the Blue Alliance origin.
        // It complies natively with PathPlanner and CTRE for the 2026 field.
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            double allianceValue = (alliance.get() == Alliance.Red) ? 0.0 : 1.0;
            m_limelightTable.getEntry("alliance").setDouble(allianceValue);
        }

        updateOdometry();

        SmartDashboard.putNumber("Shooting/Dist to goal", getOdometryDistanceMeters());
        SmartDashboard.putNumber("Shooting/Downrange to goal", getOdometryDownrangeMeters());
        SmartDashboard.putBoolean("Shooting/Is Aligned", isOdometryAligned());
        
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Fallback Distance", getDistanceMeters_Fallback());
    }
}
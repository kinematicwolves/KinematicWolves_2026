package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionProfile;
import frc.robot.Constants.FieldConstants;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable m_limelightTable;
    private final PIDController m_aimController;
    private final Swerve m_swerve; // We need Swerve to update Odometry

    public VisionSubsystem(Swerve swerve) {
        m_swerve = swerve;
        m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionProfile.kLimelightName);
        
        m_aimController = new PIDController(
            VisionProfile.kP_Align, 
            VisionProfile.kI_Align, 
            VisionProfile.kD_Align
        );
        
        m_aimController.setSetpoint(0.0);
        m_aimController.setTolerance(VisionProfile.kAlignToleranceDegrees);
    }

    /* ========================================================= */
    /* RAW LIMELIGHT DATA                                        */
    /* ========================================================= */

    public boolean hasTarget() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    /* ========================================================= */
    /* PROCESSED DATA FOR SCORING (Keep as Fallbacks)            */
    /* ========================================================= */

    public double getDistanceMeters_Fallback() {
        if (!hasTarget()) return 0.0;
        double targetOffsetAngle_Vertical = m_limelightTable.getEntry("ty").getDouble(0.0);
        double angleToGoalRadians = Math.toRadians(FieldConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical);
        double heightDifference = FieldConstants.kTargetCenterHeightMeters - FieldConstants.kLimelightMountHeightMeters;
        return heightDifference / Math.tan(angleToGoalRadians);
    }

    public double getSteeringFeedback_Fallback() {
        if (!hasTarget()) return 0.0; 
        return -m_aimController.calculate(getTx());
    }

    /* ========================================================= */
    /* MEGATAG ODOMETRY UPDATES                                  */
    /* ========================================================= */

    private void updateOdometry() {
        if (!hasTarget()) return;

        // Limelight's botpose_wpiblue array gives us: [x, y, z, roll, pitch, yaw, latency_ms]
        double[] botpose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        
        if (botpose.length == 7) {
            Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
            
            // Calculate exact timestamp by subtracting Limelight latency from the current FPGA time
            double latencySeconds = botpose[6] / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latencySeconds;

            // Feed the vision data to the Swerve Drive!
            m_swerve.addVisionMeasurement(visionPose, timestamp);
        }
    }

    @Override
    public void periodic() {
        // Tell Limelight our current Alliance for MegaTag filtering
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            double allianceValue = (alliance.get() == Alliance.Red) ? 0.0 : 1.0;
            m_limelightTable.getEntry("alliance").setDouble(allianceValue);
        }

        // Continually update our Swerve Odometry using the Limelight Pose
        updateOdometry();

        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Fallback Distance", getDistanceMeters_Fallback());
    }
}
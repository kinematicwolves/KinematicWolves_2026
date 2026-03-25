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

    // Controller for Limelight Crosshair (Fallback)
    private final PIDController m_limelightAimController;
    
    // Controller for Odometry/Pose Aiming (Primary)
    private final PIDController m_odometryAimController;

    public Vision(Swerve swerve) {
        m_swerve = swerve;
        m_limelightTable = NetworkTableInstance.getDefault().getTable(VisionProfile.kLimelightName);
        
        // Setup Limelight Fallback Controller
        m_limelightAimController = new PIDController(
            VisionProfile.kP_Align, 
            VisionProfile.kI_Align, 
            VisionProfile.kD_Align
        );
        m_limelightAimController.setSetpoint(0.0);
        m_limelightAimController.setTolerance(VisionProfile.kAlignToleranceDegrees);

        // Setup Odometry Controller (Previously in RobotContainer)
        // TODO: Move these PID constants to VisionProfile in Constants.java
        m_odometryAimController = new PIDController(5.0, 0.0, 0.1); 
        m_odometryAimController.enableContinuousInput(-Math.PI, Math.PI);
        m_odometryAimController.setTolerance(Units.degreesToRadians(2.0));
    }

    /* ========================================================= */
    /* POSE-BASED SCORING (PRIMARY METHOD)                       */
    /* ========================================================= */

    /** Helper: Which Hub are we shooting at? */
    public Translation2d getTargetHub() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? FieldConstants.kRedHub : FieldConstants.kBlueHub;
    }

    /** Helper: Calculate distance from Robot Pose to the Hub */
    public double getOdometryDistanceMeters() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        return robotPos.getDistance(getTargetHub());
    }

    /** Helper: Calculate the rotational speed needed to face the Hub */
    public double getOdometryAimRate() {
        Translation2d robotPos = m_swerve.getPose().getTranslation();
        Translation2d target = getTargetHub();
        
        // Calculate the angle from the robot to the target
        Rotation2d targetHeading = target.minus(robotPos).getAngle();
        
        // BACKWARDS OFFSET: Because your shooter is on the BACK
        targetHeading = targetHeading.plus(Rotation2d.fromDegrees(180));

        // Calculate the PID output based on our current rotation vs target rotation
        return m_odometryAimController.calculate(
            m_swerve.getPose().getRotation().getRadians(), 
            targetHeading.getRadians()
        );
    }

    /** Checks if Odometry rotation is locked onto the target */
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

        double[] botpose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        
        if (botpose.length == 7) {
            Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
            double latencySeconds = botpose[6] / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latencySeconds;

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

        // Continually update our Swerve Odometry
        updateOdometry();

        // TELEMETRY (Use these for your Tuning!)
        SmartDashboard.putNumber("Shooting/Odometry Distance", getOdometryDistanceMeters());
        SmartDashboard.putBoolean("Shooting/Is Aligned", isOdometryAligned());
        
        // Fallback telemetry
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Fallback Distance", getDistanceMeters_Fallback());
    }
}
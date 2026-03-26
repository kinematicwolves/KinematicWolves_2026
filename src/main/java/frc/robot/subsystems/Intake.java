package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeProfile;
import frc.robot.generated.TunerConstants;

/**
 * Manages the intake pivot (Motion Magic) and the floor-loading rollers.
 */
public class Intake extends SubsystemBase {
    private final TalonFX m_pivotMaster;
    private final TalonFX m_pivotFollower;
    private final WPI_TalonSRX m_roller;

    // Motion Magic request for smooth, trapezoidal profile movement
    private final MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    public Intake() {
        m_pivotMaster = new TalonFX(IntakeProfile.kPivotMasterID, TunerConstants.kCANBus);
        m_pivotFollower = new TalonFX(IntakeProfile.kPivotFollowerID, TunerConstants.kCANBus);
        m_roller = new WPI_TalonSRX(IntakeProfile.kRollerID);
        configureHardware();
    }

    /** Configures pivot PID, Motion Magic constraints, and roller inversions */
    private void configureHardware() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Hold intake up when disabled
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Pivot PID Gains
        pivotConfig.Slot0.kP = IntakeProfile.kPivotP;
        pivotConfig.Slot0.kI = IntakeProfile.kPivotI;
        pivotConfig.Slot0.kD = IntakeProfile.kPivotD;

        // Motion Magic: Limits speed and acceleration for a smooth "cruise"
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeProfile.kPivotMaxVelocity;
        pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeProfile.kPivotMaxAcceleration;

        m_pivotMaster.getConfigurator().apply(pivotConfig);
        
        // Set up the second pivot motor to perfectly mirror the master
        m_pivotFollower.setControl(new Follower(m_pivotMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        m_roller.configFactoryDefault();
        m_roller.setNeutralMode(NeutralMode.Coast);
        m_roller.setInverted(true); // Spin inward to pick up game pieces
    }

    /** Returns true if pivot is within tolerance of the "Down" position */
    public boolean isIntakeDown() {
        return Math.abs(m_pivotMaster.getPosition().getValueAsDouble() - IntakeProfile.kPivotDownPosition) 
               < IntakeProfile.kPivotTolerance;
    }

    /** Sends a position target to the master pivot motor */
    public void setPivotPosition(double rotations) {
        m_pivotMaster.setControl(m_pivotRequest.withPosition(rotations));
    }

    /** Direct voltage control for the intake rollers */
    public void setRollerVoltage(double volts) {
        m_roller.setVoltage(volts);
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /** Move pivot to a specific rotation value */
    public Command setPivotCommand(double rotations) {
        return run(() -> setPivotPosition(rotations));
    }

    /** Run rollers at voltage; stops rollers when command ends */
    public Command runRollersCommand(double volts) {
        return run(() -> setRollerVoltage(volts)).finallyDo(() -> setRollerVoltage(0));
    }

    /** * Full Intake Sequence: Lowers the pivot AND only runs rollers once 
     * physically in position to avoid ground-scraping or jams.
     */
    public Command deploySequenceCommand() {
        return run(() -> {
            setPivotPosition(IntakeProfile.kPivotDownPosition);

            if (isIntakeDown()) {
                setRollerVoltage(IntakeProfile.kRollerVoltage);
            } else {
                setRollerVoltage(0.0); // Safety: Keep rollers off while moving
            }
        })
        .finallyDo(() -> setRollerVoltage(0.0)) // Stop rollers on release
        .withName("IntakeDeploySequence");
    }

    /** Reverses rollers to clear jams or spit out game pieces */
    public Command exhaustCommand() {
        return runRollersCommand(IntakeProfile.kExhaustVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Pos", m_pivotMaster.getPosition().getValueAsDouble());
    }
}
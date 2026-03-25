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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeProfile;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
    private final TalonFX m_pivotMaster;
    private final TalonFX m_pivotFollower;
    private final WPI_TalonSRX m_roller;
    private final MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    public Intake() {
        m_pivotMaster = new TalonFX(IntakeProfile.kPivotMasterID, TunerConstants.kCANBus);
        m_pivotFollower = new TalonFX(IntakeProfile.kPivotFollowerID, TunerConstants.kCANBus);
        m_roller = new WPI_TalonSRX(IntakeProfile.kRollerID);
        configureHardware();
    }

    private void configureHardware() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Clockwise_Positive = Inverted
        // CounterClockwise_Positive = Default
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        pivotConfig.Slot0.kP = IntakeProfile.kPivotP;
        pivotConfig.Slot0.kI = IntakeProfile.kPivotI;
        pivotConfig.Slot0.kD = IntakeProfile.kPivotD;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeProfile.kPivotMaxVelocity;
        pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeProfile.kPivotMaxAcceleration;

        // Apply config to the Master
        m_pivotMaster.getConfigurator().apply(pivotConfig);
        
        // The follower automatically spins the opposite way of the master
        m_pivotFollower.setControl(new Follower(m_pivotMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        m_roller.configFactoryDefault();
        m_roller.setNeutralMode(NeutralMode.Coast);
        m_roller.setInverted(false);
    }

    public boolean isIntakeDown() {
        return Math.abs(m_pivotMaster.getPosition().getValueAsDouble() - IntakeProfile.kPivotDownPosition) 
               < IntakeProfile.kPivotTolerance;
    }

    public void setPivotPosition(double rotations) {
        m_pivotMaster.setControl(m_pivotRequest.withPosition(rotations));
    }

    public void setRollerVoltage(double volts) {
        m_roller.setVoltage(volts);
    }

    // Simple position setting
    public Command setPivotCommand(double rotations) {
        return run(() -> setPivotPosition(rotations));
    }

    // Factory for roller control
    public Command runRollersCommand(double volts) {
        return run(() -> setRollerVoltage(volts)).finallyDo(() -> setRollerVoltage(0));
    }

    public Command deploySequenceCommand() {
        return this.setPivotCommand(IntakeProfile.kPivotDownPosition)
            .alongWith(
                Commands.waitUntil(this::isIntakeDown)
                .andThen(this.runRollersCommand(IntakeProfile.kRollerVoltage))
            ).withName("IntakeDeploySequence");
    }

    // Separate Exhaust Command
    public Command exhaustCommand() {
        return runRollersCommand(IntakeProfile.kExhaustVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Pos", m_pivotMaster.getPosition().getValueAsDouble());
    }
}
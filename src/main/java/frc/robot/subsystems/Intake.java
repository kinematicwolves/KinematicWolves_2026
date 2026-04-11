package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

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
    private final TalonFX m_roller;
    private final MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    public Intake() {
        m_pivotMaster = new TalonFX(IntakeProfile.kPivotMasterID, TunerConstants.kCANBus);
        m_pivotFollower = new TalonFX(IntakeProfile.kPivotFollowerID, TunerConstants.kCANBus);
        m_roller = new TalonFX(IntakeProfile.kRollerID, TunerConstants.kCANBus);
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
        m_pivotMaster.setPosition(0.0);
        
        // The follower automatically spins the opposite way of the master
        m_pivotFollower.setControl(new Follower(m_pivotMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        // m_roller.getConfigurator().apply(new TalonFXConfiguration());
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_roller.getConfigurator().apply(rollerConfig);
        // m_roller.configFactoryDefault();
        // m_roller.setNeutralMode(NeutralMode.Coast);
        // m_roller.setInverted(true);
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
    
   /**
    * sets the motor speed at a percent output
    * @param speed between -1 and 1
    */
    public void setRollerSpeed(double speed) {
    m_roller.set(speed);
    }

    // Simple position setting
    public Command setPivotCommand(double rotations) {
        return run(() -> setPivotPosition(rotations)).until(() -> isIntakeDown() == true);
    }

    // Factory for roller control
    public Command runRollersCommand(double volts) {
        return run(() -> setRollerVoltage(volts)).finallyDo(() -> setRollerVoltage(0));
    }

    /**
     * A unified sequence that safely handles the pivot and rollers 
     * without conflicting subsystem requirements.
     */
    public Command deploySequenceCommand() {
        return run(() -> {
            // 1. Always command the pivot to move to the down position
            // (Note: Replace 'setPivotPosition' with whatever your internal method is actually called)
            setPivotPosition(IntakeProfile.kPivotDownPosition);
            setRollerVoltage(11);

            // 2. Only apply voltage to the rollers IF the intake has physically reached the bottom
            if (isIntakeDown()) {
                setRollerVoltage(IntakeProfile.kRollerVoltage);
                System.out.println("Intake is down");
                System.out.println(IntakeProfile.kRollerVoltage);
            } else {
                setRollerVoltage(0.0); // Keep them off while traveling
                System.out.println("Intake is up");

            }
        })
        .finallyDo(() -> {
            // Safety stop for the rollers when the driver lets go of the trigger
            setRollerVoltage(0.0);
        })
        .withName("IntakeDeploySequence");
    }

    // Separate Exhaust Command
    public Command exhaustCommand() {
        return runRollersCommand(IntakeProfile.kExhaustVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Pos", m_pivotMaster.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Pivot down?", isIntakeDown());
        SmartDashboard.putNumber("Pivot Target", m_pivotMaster.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("roller", m_roller.getClosedLoopReference().getValueAsDouble());

    }
}
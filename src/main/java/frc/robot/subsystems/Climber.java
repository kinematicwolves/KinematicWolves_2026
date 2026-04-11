package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberProfile;
import frc.robot.generated.TunerConstants;

public class Climber extends SubsystemBase {

    private final TalonFX m_leftClimber;
    private final TalonFX m_rightClimber;

    // Control Requests
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut m_manualRequest = new VoltageOut(0);

    public Climber() {
        m_leftClimber = new TalonFX(ClimberProfile.kClimberLeftID, TunerConstants.kCANBus);
        m_rightClimber = new TalonFX(ClimberProfile.kClimberRightID, TunerConstants.kCANBus);

        configureHardware();
    }

    private void configureHardware() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // --- ADD SOFT LIMITS HERE ---
        // Forward = Extension, Reverse = Retraction
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberProfile.kMaxHeight; 
        
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberProfile.kHomePosition; 

        // PID & Motion Magic Profile
        config.Slot0.kP = ClimberProfile.kClimberP;
        config.Slot0.kV = ClimberProfile.kClimberV;
        config.MotionMagic.MotionMagicCruiseVelocity = ClimberProfile.kMaxVelocity;
        config.MotionMagic.MotionMagicAcceleration = ClimberProfile.kMaxAcceleration;

        // Apply config with specific inverts
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftClimber.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        m_rightClimber.getConfigurator().apply(config);
    }

    /* ========================================================= */
    /* LOGIC METHODS                                             */
    /* ========================================================= */

    public void setPosition(double targetRotations) {
        m_leftClimber.setControl(m_positionRequest.withPosition(targetRotations));
        m_rightClimber.setControl(m_positionRequest.withPosition(targetRotations));
    }

    /**
     * Checks if both climbers are within the acceptable range of the target.
     */
    public boolean isAtPosition(double targetRotations) {
        double leftError = Math.abs(m_leftClimber.getPosition().getValueAsDouble() - targetRotations);
        double rightError = Math.abs(m_rightClimber.getPosition().getValueAsDouble() - targetRotations);
        
        return (leftError <= ClimberProfile.kTolerance) && (rightError <= ClimberProfile.kTolerance);
    }
    public void setManualVoltage(double volts) {
        m_leftClimber.setControl(m_manualRequest.withOutput(volts));
        m_rightClimber.setControl(m_manualRequest.withOutput(volts));
    }

    public void stop() {
        m_leftClimber.stopMotor();
        m_rightClimber.stopMotor();
    }

    /**
     * Resets the internal encoders to 0. 
     */
    public void zeroEncoders() {
        m_leftClimber.setPosition(0);
        m_rightClimber.setPosition(0);
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /**
     * Extends the climber to the maximum height. 
     * The command finishes automatically when it reaches the target range.
     */
    public Command extendCommand() {
        return run(() -> setPosition(ClimberProfile.kMaxHeight))
               .until(() -> isAtPosition(ClimberProfile.kMaxHeight))
               .withName("ClimberExtend");
    }

    /**
     * Retracts the climber to pull the robot off the ground.
     * The command finishes automatically when it reaches the target range.
     */
    public Command climbCommand() {
        return run(() -> setPosition(ClimberProfile.kHomePosition))
               .until(() -> isAtPosition(ClimberProfile.kHomePosition))
               .withName("ClimberRetract");
    }

    /**
     * Manual override using the operator's joystick.
     */
    public Command manualOverrideCommand(DoubleSupplier voltageSupplier) {
        return run(() -> setManualVoltage(voltageSupplier.getAsDouble()))
               .finallyDo(this::stop)
               .withName("ClimberManual");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Left Pos", m_leftClimber.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/Right Pos", m_rightClimber.getPosition().getValueAsDouble());
    }
}
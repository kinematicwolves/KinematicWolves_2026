package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerProfile;

public class Indexer extends SubsystemBase {

    private final WPI_TalonSRX m_hopper;
    private final SparkMax m_kicker;

    public Indexer() {
        m_hopper = new WPI_TalonSRX(IndexerProfile.kHopperID);
        m_kicker = new SparkMax(IndexerProfile.kKickerID, MotorType.kBrushless);

        configureHardware();
    }

    private void configureHardware() {
        /* --- HOPPER CONFIGURATION (Talon SRX) --- */
        m_hopper.configFactoryDefault();
        m_hopper.setNeutralMode(NeutralMode.Coast); // Coast allows balls to settle
        m_hopper.configContinuousCurrentLimit(IndexerProfile.kHopperCurrentLimit);
        m_hopper.enableCurrentLimit(true);
        m_hopper.setInverted(false); // Change to true if the motor spins the wrong way

        /* --- KICKER CONFIGURATION --- */
        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        
        kickerConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IndexerProfile.kKickerCurrentLimit);

        m_kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ========================================================= */
    /* METHODS                                            */
    /* ========================================================= */

    /**
     * Directly sets the voltages of the hopper and kicker.
     */
    public void setVoltages(double hopperVolts, double kickerVolts) {
        m_hopper.setVoltage(hopperVolts);
        m_kicker.setVoltage(kickerVolts);
    }

    /**
     * Safely stops all indexer motors.
     */
    public void stop() {
        setVoltages(0.0, 0.0);
    }

    /* ========================================================= */
    /* COMMANDS                                                  */
    /* ========================================================= */

    /**
     * Runs both the hopper and the kicker forward to feed the shooter.
     * Stops automatically when the command is interrupted.
     */
    public Command feedShooterCommand() {
        return run(() -> setVoltages(IndexerProfile.kHopperVoltage, IndexerProfile.kKickerVoltage))
               .finallyDo(this::stop)
               .withName("IndexerFeed");
    }

    /**
     * Runs both motors in reverse to clear a jam or eject a wrong piece.
     */
    public Command reverseIndexerCommand() {
        return run(() -> setVoltages(IndexerProfile.kReverseVoltage, IndexerProfile.kReverseVoltage))
               .finallyDo(this::stop)
               .withName("IndexerReverse");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker RPM", m_kicker.getEncoder().getVelocity());
    }
}
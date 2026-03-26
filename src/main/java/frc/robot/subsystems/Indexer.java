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

/**
 * Handles the internal transport of game pieces.
 * Hopper: Moves balls from the intake area into the internal queue.
 * Kicker: High-speed motor that "kicks" the ball into the shooter flywheels.
 */
public class Indexer extends SubsystemBase {

    private final WPI_TalonSRX m_hopper;
    private final SparkMax m_kicker;

    public Indexer() {
        m_hopper = new WPI_TalonSRX(IndexerProfile.kHopperID);
        m_kicker = new SparkMax(IndexerProfile.kKickerID, MotorType.kBrushless);

        configureHardware();
    }

    /** Sets up current limits and motor orientations */
    private void configureHardware() {
        /* --- HOPPER (Talon SRX) --- */
        m_hopper.configFactoryDefault();
        m_hopper.setNeutralMode(NeutralMode.Coast); // Allow balls to settle naturally
        m_hopper.configContinuousCurrentLimit(IndexerProfile.kHopperCurrentLimit);
        m_hopper.enableCurrentLimit(true);
        m_hopper.setInverted(true); 

        /* --- KICKER (SparkMax) --- */
        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        
        kickerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake) // Instant stop to prevent "double-firing"
            .smartCurrentLimit(IndexerProfile.kKickerCurrentLimit);

        m_kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ========================================================= */
    /* LOGIC METHODS                                             */
    /* ========================================================= */

    /** Applies direct voltage to both stages of the indexer */
    public void setVoltages(double hopperVolts, double kickerVolts) {
        m_hopper.setVoltage(hopperVolts);
        m_kicker.setVoltage(kickerVolts);
    }

    /** Forces all indexer motors to zero */
    public void stop() {
        setVoltages(0.0, 0.0);
    }

    /* ========================================================= */
    /* COMMAND FACTORIES                                         */
    /* ========================================================= */

    /** Runs the entire indexer forward to feed the shooter; stops on release */
    public Command feedShooterCommand() {
        return run(() -> setVoltages(IndexerProfile.kHopperVoltage, IndexerProfile.kKickerVoltage))
               .finallyDo(this::stop)
               .withName("IndexerFeed");
    }

    /** Reverses both motors to clear jams or spit out bad pieces */
    public Command reverseIndexerCommand() {
        return run(() -> setVoltages(IndexerProfile.kReverseVoltage, IndexerProfile.kReverseVoltage))
               .finallyDo(this::stop)
               .withName("IndexerReverse");
    }

    @Override
    public void periodic() {
        // Monitor kicker speed to see if it's bogging down during shots
        SmartDashboard.putNumber("Indexer/Kicker RPM", m_kicker.getEncoder().getVelocity());
    }
}
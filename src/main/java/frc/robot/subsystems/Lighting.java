package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Lighting extends SubsystemBase {
    private final CANdle m_candle;
    
    // --- THE FIX: CANdle Addressing ---
    // The CANdle has 8 built-in LEDs (Indices 0-7).
    // Your external strip starts at Index 8!
    private final int kStripStart = 8; 
    private final int kNumLeds = 65; 
    private final int kStripEnd = kStripStart + kNumLeds - 1; // Index 67

    public enum LEDState {
        IDLE,
        HAS_TARGET,
        READY_TO_FIRE
    }

    private LEDState m_currentState = LEDState.IDLE;
    private Alliance m_lastAlliance = null;

    /* ========================================================= */
    /* PHOENIX 6 CONTROL REQUESTS                                */
    /* ========================================================= */
    
    // Pass the calculated Start and End indices to target ONLY the external strip
    private final StrobeAnimation m_readyAnim = new StrobeAnimation(kStripStart, kStripEnd)
        .withColor(new RGBWColor(0, 255, 0, 0))
        .withFrameRate(50);

    private final SolidColor m_targetColor = new SolidColor(kStripStart, kStripEnd)
        .withColor(new RGBWColor(255, 100, 0, 0)); 

    private final ColorFlowAnimation m_redIdle = new ColorFlowAnimation(kStripStart, kStripEnd)
        .withColor(new RGBWColor(255, 0, 0, 0))
        .withFrameRate(150);

    private final ColorFlowAnimation m_blueIdle = new ColorFlowAnimation(kStripStart, kStripEnd)
        .withColor(new RGBWColor(0, 0, 255, 0))
        .withFrameRate(150);

    public Lighting() {
        // IMPORTANT: If your CANdle is on a CANivore, you must add the CANbus string here!
        // Example: m_candle = new CANdle(canId, "carnivore");
        m_candle = new CANdle(5, TunerConstants.kCANBus);
        
        // --- THE FIX: Let Tuner X Handle Configuration ---
        // In Phoenix 6, you should set your LED Strip Type (GRB) and Brightness 
        // permanently inside the Phoenix Tuner X application. 
        // We do not configure it here to prevent overwriting your Tuner settings!
    }

    public void setState(LEDState state) {
        if (m_currentState == state) return; 
        m_currentState = state;

        switch (state) {
            case READY_TO_FIRE:
                m_candle.setControl(m_readyAnim);
                break;
                
            case HAS_TARGET:
                m_candle.setControl(m_targetColor);
                break;
                
            case IDLE:
            default:
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                m_candle.setControl(isRed ? m_redIdle : m_blueIdle);
                break;
        }
    }

    @Override
    public void periodic() {
        // Refresh the Idle animation if the Driver Station switches our Alliance
        if (m_currentState == LEDState.IDLE) {
            Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            if (currentAlliance != m_lastAlliance) {
                m_lastAlliance = currentAlliance;
                m_currentState = null; 
                setState(LEDState.IDLE);
            }
        }
    }
}
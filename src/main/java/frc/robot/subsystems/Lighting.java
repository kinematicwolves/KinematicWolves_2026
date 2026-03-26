package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightingProfile;
import frc.robot.generated.TunerConstants;

public class Lighting extends SubsystemBase {
    private final CANdle m_candle;
    
    // Pulled directly from Constants
    private final int kEndIndex = LightingProfile.kStartIndex + LightingProfile.kNumLeds - 1;

    public enum LEDState {
        DISABLED,      // Rainbow
        ALLIANCE,      // Solid Red or Blue
        HUB_ACTIVE     // Solid Green
    }

    private LEDState m_currentState = null;
    private Alliance m_lastAlliance = null;

    /* ========================================================= */
    /* PHOENIX 6 CONTROL REQUESTS                                */
    /* ========================================================= */
    
    private final RainbowAnimation m_disabledAnim = new RainbowAnimation(LightingProfile.kStartIndex, kEndIndex)
        .withBrightness(LightingProfile.kRainbowBrightness) 
        .withFrameRate(LightingProfile.kRainbowFrameRate); 

    private final SolidColor m_hubActiveColor = new SolidColor(LightingProfile.kStartIndex, kEndIndex)
        .withColor(new RGBWColor(0, 255, 0, 0)); // Green
        
    private final SolidColor m_redAlliance = new SolidColor(LightingProfile.kStartIndex, kEndIndex)
        .withColor(new RGBWColor(255, 0, 0, 0)); // Red
        
    private final SolidColor m_blueAlliance = new SolidColor(LightingProfile.kStartIndex, kEndIndex)
        .withColor(new RGBWColor(0, 0, 255, 0)); // Blue

    public Lighting() {
        m_candle = new CANdle(LightingProfile.kCandleID, TunerConstants.kCANBus);
    }

    /* ========================================================= */
    /* STATE CONTROL                                             */
    /* ========================================================= */

    private void setState(LEDState state) {
        if (m_currentState == state) return; 
        m_currentState = state;

        switch (state) {
            case DISABLED:
                m_candle.setControl(m_disabledAnim);
                break;
            case HUB_ACTIVE:
                m_candle.setControl(m_hubActiveColor);
                break;
            case ALLIANCE:
            default:
                boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                m_candle.setControl(isRed ? m_redAlliance : m_blueAlliance);
                break;
        }
    }

    /* ========================================================= */
    /* 2026 FIELD TIMING LOGIC                                   */
    /* ========================================================= */

    /**
     * Calculates the exact state of the Hub at a SPECIFIC match time.
     */
    private boolean isHubActiveAtTime(double matchTime) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // 2026 Hub Shift Timings
        if (matchTime > 130) return true;
        else if (matchTime > 105) return shift1Active;
        else if (matchTime > 80) return !shift1Active;
        else if (matchTime > 55) return shift1Active;
        else if (matchTime > 30) return !shift1Active;
        else return true; // Endgame
    }

    /**
     * Uses the dual-predictive logic to give drivers a heads up.
     * Uses timing constants from LightingProfile.
     */
    private boolean getPredictiveHubState() {
        if (!DriverStation.isTeleopEnabled()) return isHubActiveAtTime(DriverStation.getMatchTime());

        double currentMatchTime = DriverStation.getMatchTime();
        boolean isCurrentlyActive = isHubActiveAtTime(currentMatchTime);

        if (isCurrentlyActive) {
            // We are active. Look into the future to see if we are about to deactivate.
            return isHubActiveAtTime(currentMatchTime - LightingProfile.kHubDeactivateWarningSec);
        } else {
            // We are inactive. Look into the future to see if we are about to activate.
            return isHubActiveAtTime(currentMatchTime - LightingProfile.kHubActivateWarningSec);
        }
    }

    /* ========================================================= */
    /* SELF-MANAGING PERIODIC LOOP                               */
    /* ========================================================= */

    @Override
    public void periodic() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Force a state refresh if the alliance color gets flipped on the dashboard
        if (m_currentState == LEDState.ALLIANCE && currentAlliance != m_lastAlliance) {
            m_currentState = null; 
            m_lastAlliance = currentAlliance;
        }

        // Master State Machine
        if (DriverStation.isDisabled()) {
            setState(LEDState.DISABLED); // Rainbow
        } else {
            if (getPredictiveHubState()) {
                setState(LEDState.HUB_ACTIVE); // Green
            } else {
                setState(LEDState.ALLIANCE); // Red/Blue
            }
        }
    }
}
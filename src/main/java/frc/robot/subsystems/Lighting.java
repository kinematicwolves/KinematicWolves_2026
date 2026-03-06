// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANdleSegment;
import frc.robot.Constants.LEDProfile;
import frc.robot.generated.TunerConstants;

public class Lighting extends SubsystemBase {
    public final CANdle candle = new CANdle(5, TunerConstants.kCANBus);

    /** Creates a new Lighting. */
    public Lighting() {
        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = LEDProfile.BrightnessScalar;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Clears all color and animations from the segment. 
     * Best practice is to call this before applying a new color / animation to the segment.
     * @param segment the segment to clear
     */
    private void flushSegment(CANdleSegment segment) {
        this.candle.setControl(new SolidColor(segment.getStartLed(), segment.getEndLed()).withColor(LEDProfile.off));
        setSegmentEmptyAnimation(segment);
    }

    /**
     * Clears the animation slot on the CANdle for the segment.
     * Bes Practice is to call this before applying a new animation to the segment.
     * At least in my testing.
     * @param segment The segment to clear
     */
    private void setSegmentEmptyAnimation(CANdleSegment segment) {
        this.candle.setControl(new EmptyAnimation(segment.getSlot()));
    }

    /**
     * Applies a color to the segment
     * @param segment the segment to color
     * @param color the color to apply to the segment
     */
    public void setSegmentColor(CANdleSegment segment, RGBWColor color) {
        // first flush the segment with all off
        flushSegment(segment);
        // then apply the passed color
        this.candle.setControl(new SolidColor(segment.getStartLed(), segment.getEndLed()).withColor(color));
    }

    public void setSegmentFireAnimation(CANdleSegment segment, double cooling, double sparking) {
        flushSegment(segment);
        this.candle.setControl(
            new FireAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withCooling(cooling)
            .withSparking(sparking)
        );
    }

    public void setSegmentLarsonAnimation(CANdleSegment segment, RGBWColor color, int size) {
        flushSegment(segment);
        this.candle.setControl(
            new LarsonAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withColor(color)
            .withSize(size)
            .withBounceMode(LarsonBounceValue.Front)
        );
    }

    public void setSegmentStrobeAnimation(CANdleSegment segment, RGBWColor color, double hz) {
        flushSegment(segment);
        this.candle.setControl(
            new StrobeAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withColor(color)
            .withFrameRate(hz)
        );
    }

    public void setSegmentRainbowAnimation(CANdleSegment segment) {
        flushSegment(segment);
        this.candle.setControl(
            new RainbowAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
        );
    }

    public void setSegmentRgbFadeAnimation(CANdleSegment segment) {
        flushSegment(segment);
        this.candle.setControl(
            new RgbFadeAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
        );
    }

    public void setSegmentTwinkleAnimation(CANdleSegment segment, RGBWColor color) {
        flushSegment(segment);
        this.candle.setControl(
            new TwinkleAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
        );
    }

    public void setSegmentColorFlowAnimation(CANdleSegment segment, RGBWColor color) {
        flushSegment(segment);
        this.candle.setControl(
            new ColorFlowAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withColor(color)
        );
    }

    public void setSegmentSingleFadeAnimation(CANdleSegment segment, RGBWColor color) {
        flushSegment(segment);
        this.candle.setControl(
            new SingleFadeAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withColor(color)
        );
    }

    public void setSegmentTwinkleOffAnimation(CANdleSegment segment, RGBWColor color) {
        flushSegment(segment);
        this.candle.setControl(
            new TwinkleOffAnimation(segment.getStartLed(), segment.getEndLed())
            .withSlot(segment.getSlot())
            .withColor(color)
        );
    }
}

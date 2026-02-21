// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** A segment of leds controlled by CANdle. */
public class CANdleSegment {
    private int startIndex;
    private int endIndex;
    private int slot;

    /**
     * A segment of leds controlled by CANdle.
     * @param start The starting led of the segment. 0-7 are on the CANdle, 8-399 are downstream of the candle
     * @param end The ending led of the segment. 0-7 are on the CANdle, 8-399 are downstream of the candle
     * @param slot The animation slot for this segment on the CANdle. 0-7 supported.
     */
    public CANdleSegment(int start, int end, int slot) {
        this.startIndex = start;
        this.endIndex = end;
        this.slot = slot;
    }

    /**
     * 
     * @return The start index of the segment
     */
    public int getStartLed() {
        return startIndex;
    }

    /**
     * 
     * @return The end index of the segment
     */
    public int getEndLed() {
        return endIndex;
    }

    /**
     * 
     * @return The animation slot for the CANdle
     */
    public int getSlot() {
        return slot;
    }
}

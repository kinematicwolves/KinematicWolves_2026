// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/** Add your docs here. */

public class ShotParams implements Interpolatable<ShotParams> {

    public final double hoodRotations;
    public final double rpm;

    public ShotParams(double hoodRotations, double rpm) {
        this.hoodRotations = hoodRotations;
        this.rpm = rpm;
    }

    @Override
    public ShotParams interpolate(ShotParams endValue, double t) {
        return new ShotParams(
            MathUtil.interpolate(this.hoodRotations, endValue.hoodRotations, t),
            MathUtil.interpolate(this.rpm, endValue.rpm, t)
        );
    }
}
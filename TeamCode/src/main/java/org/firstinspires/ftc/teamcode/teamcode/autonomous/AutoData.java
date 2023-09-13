package org.firstinspires.ftc.teamcode.teamcode.autonomous;

public class AutoData {

    public static final double TICKS_PER_REV_DEAD = 8192;

    public static double DEADWHEEL_DIAM_MM = 35;
    public static double DEADWHEEL_DIAM_IN = DEADWHEEL_DIAM_MM * 0.0393701;

    public static double encoderTicksToInches(double ticks) {
        return DEADWHEEL_DIAM_IN * Math.PI * ticks / TICKS_PER_REV_DEAD;
    }

    public static double encoderInchesToTicks(double inches) {
        return (inches * TICKS_PER_REV_DEAD) / (DEADWHEEL_DIAM_IN * Math.PI);
    }
}

package org.firstinspires.ftc.teamcode.teamcode;

public class teleOp_Constants {

    private double maxSpeed;

    private double clawOpen;
    private double clawClose;

    public teleOp_Constants() {
        maxSpeed = 0.7;

        clawOpen = 1;
        clawClose = 0.7;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getClawClose() {
        return clawClose;
    }

    public double getClawOpen() {
        return clawOpen;
    }
}

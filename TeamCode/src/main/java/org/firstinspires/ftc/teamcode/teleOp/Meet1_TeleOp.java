/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.driveTools.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.driveTools.StandardTrackingWheelLocalizer;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// WHAT IS WHAT
// gamepad.2 is "weapons" - claw (x- close, y- open) - slide (a- up, b- down)
// gamepad.1 is driving - motors dual-stick drive, right bumper for slow mode


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Normal Drive Mode", group = "Linear Opmode")
public class Meet1_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Motors + Encoders
    SampleMecanumDrive drive = null;
    StandardTrackingWheelLocalizer encoders = null;

    // Linear Slide
    private DcMotor smallGear = null;
    private DcMotor bigGear = null;

    double bigGearPower;
    double smallGearPower;

    int operationMode = 2;

    private Servo Servo1 = null;
    // Used for vert linear slide
    private boolean upperBoundHit = false;
    private boolean lowerBoundHit = false;
    // MAX_TICKS is the value at the top (don't raise up more than this)
    // MIN_TICKS is the value at the bottom (don't wind up more than this)
    final int MAX_TICKS = 3000;
    final int MIN_TICKS = 0;

    @Override
    public void runOpMode() {

        // Method to assign and initialize the hardware
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            driveCode(gamepad1);
            controlClaw(gamepad2);
            controlLinearSlide(gamepad2);


        }
    }


    private void driveCode(Gamepad gp) {

        // Drive Constants
        double maxSpeed = 0.7;


        // Input Calculations
        float x = gp.left_stick_x;
        float y = -gp.left_stick_y;

        double lfp = Range.clip(x + y, -maxSpeed, maxSpeed);
        double lbp = Range.clip(y - x, -maxSpeed, maxSpeed);

        double rfp = Range.clip(y - x, -maxSpeed, maxSpeed);
        double rbp = Range.clip(x + y, -maxSpeed, maxSpeed);


        // Fast rotation
        float rotSpeed;
        if (gp.right_stick_button)
            rotSpeed = 1;
        else
            rotSpeed = 2;


        // Slow mode
        boolean slowModeActive = gp.right_bumper;
        double speedModifier;
        if (slowModeActive) {
            speedModifier = .4;
            setMotorsBreakMode();
        } else {
            speedModifier = 1;
            setMotorsFloatMode();
        }


        // Rotational offset code factoring in precalculated drive code
        float rot = -gp.right_stick_x;

        lfp = Range.clip(lfp - rot / rotSpeed, -1.0, 1.0);
        lbp = Range.clip(lbp - rot / rotSpeed, -1.0, 1.0);

        rfp = Range.clip(rfp + rot / rotSpeed, -1.0, 1.0);
        rbp = Range.clip(rbp + rot / rotSpeed, -1.0, 1.0);

        // Send calculated power to wheels
        drive.setMotorPowers(lfp * speedModifier, lbp * speedModifier, rbp * speedModifier, rfp * speedModifier);

        // Printing to console
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Power", "lf (%.2f), rb (%.2f), lb (%.2f), rf (%.2f)", lfp, rbp, lbp, rfp);
        telemetry.addData("Speed Modifier", "Master (%.2f), Rotational (%.2f)", speedModifier, rotSpeed);
        telemetry.addData("Left Encoder", encoders.getWheelPositions().get(0));
        telemetry.addData("Right Encoder", encoders.getWheelPositions().get(1));
        telemetry.addData("Front Encoder", encoders.getWheelPositions().get(2));
        telemetry.update();
    }

    public void initializeHardware() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive = new SampleMecanumDrive(hardwareMap);
        Servo1 = hardwareMap.get(Servo.class, "servo1");


        // Setting the motor encoder position to zero
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensuring the motors get the instructions
        sleep(100);

        // This makes sure the motors are not using encoders (we don't use them)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Encoders
        encoders = new StandardTrackingWheelLocalizer(hardwareMap);

        setMotorsBreakMode();

        // Linear Slide
        smallGear = hardwareMap.get(DcMotor.class, "smallGear");
        bigGear = hardwareMap.get(DcMotor.class, "bigGear");

        smallGear.setDirection(DcMotor.Direction.FORWARD);
        bigGear.setDirection(DcMotor.Direction.REVERSE);

        bigGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // DO NOT DO THIS. IT BREAKS THE MOTOR. IF YOU NEED TO RESET THE ENCODER
        bigGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        smallGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // small doesn't use encoder this is for troubleshoot

        bigGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        smallGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void controlLinearSlide(Gamepad gp) {


//        opmode 1 = small gear used for hook
//        opmode 2 = big gear used for everything
//        op mode 3 = emergency up
//        op mode 4 = emergency down

        if (gp.left_bumper) {
            operationMode = 1;
        } else if (gp.dpad_up) {
            operationMode = 3;
        } else if (gp.dpad_down) {
            operationMode = 4;
        } else if (gp.dpad_left) {
            bigGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bigGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            operationMode = 2;
        }


        switch (operationMode) {

            case 1:
                smallGearPower = Range.clip(gamepad2.left_stick_x, -1.0, 1.0);
                bigGearPower = 0;

                // If neither motor has power, break. Otherwise, both float
                if (smallGearPower == 0) {
                    slideMotorsBrakeMode();
                } else {
                    slideMotorsFloatMode();
                }

                // Send calculated power to wheels

                if (bigGear.getCurrentPosition() > MAX_TICKS || bigGear.getCurrentPosition() < MIN_TICKS) {
                    bigGearPower=0;
                }

                bigGear.setPower(0);
                smallGear.setPower(smallGearPower);
                break;

            case 2:
                bigGearPower = Range.clip(gamepad2.left_stick_x, -1.0, 1.0);
                smallGearPower = 0;

                if (bigGearPower == 0) {
                    slideMotorsBrakeMode();
                } else {
                    slideMotorsFloatMode();
                }

                if (bigGearPower > 0) {
                    // Going up

                    // If the linear slide has hit the top then the upperBoundHit becomes true
                    // We can only move the linear slide up if upperBoundHit is false
                    // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
                    if (bigGear.getCurrentPosition() >= MAX_TICKS)
                        upperBoundHit = true;
                    else if (bigGear.getCurrentPosition() < MAX_TICKS - 100)
                        upperBoundHit = false;

                    // If the current position is valid, we move the motor upwards
                    // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
                    if ((bigGear.getCurrentPosition() < MAX_TICKS - 150) && (!upperBoundHit))
                        bigGear.setPower(1);
                    else if ((bigGear.getCurrentPosition() < MAX_TICKS && (!upperBoundHit)))
                        bigGear.setPower(0.4);
                    else
                        bigGear.setPower(0.2);

                } else if (bigGearPower < 0) {
                    // Going down

                    // If the linear slide has hit the top then the upperBoundHit becomes true
                    // We can only move the linear slide up if upperBoundHit is false
                    // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
                    if (bigGear.getCurrentPosition() <= MIN_TICKS + 25)
                        lowerBoundHit = true;
                    else if (bigGear.getCurrentPosition() > MIN_TICKS)
                        lowerBoundHit = false;


                    // If the current position is valid, we move the motor upwards
                    // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
                    if ((bigGear.getCurrentPosition() > MIN_TICKS) && (!lowerBoundHit))

                        // It needs to move slower compared to how close it is to the bottom
                        if (bigGear.getCurrentPosition() > 750)
                            bigGear.setPower(-1);
                        else if (bigGear.getCurrentPosition() > 400)
                            bigGear.setPower(-.4);
                        else
                            bigGear.setPower(-.2);
                    else
                        bigGear.setPower(0);
                }

                bigGear.setPower(bigGearPower);
                smallGear.setPower(0);
                break;

            case 3:
                bigGear.setPower(.5);
                break;

            case 4:
                bigGear.setPower(-.5);
                break;
        }

        telemetry.addData("Big Gear ticks", bigGear.getCurrentPosition());
        telemetry.addData("OpMode", operationMode);
        telemetry.addData("Big Gear power", bigGear.getPower());
        telemetry.addData("Big Gear var power", bigGearPower);
        telemetry.addData("Small Gear power", smallGear.getPower());
        telemetry.addData("Big Gear Mode", bigGear.getZeroPowerBehavior());
        telemetry.addData("Small Gear Mode", smallGear.getZeroPowerBehavior());
        telemetry.update();

    }

    public void controlClaw(Gamepad gp) {
        if (gp.a) {
            Servo1.setPosition(0);
        } else if (gp.b) {
            Servo1.setPosition(0.1);
        } else if (gp.x){
            Servo1.setPosition(0.15);
        }

    }

    public void setMotorsBreakMode() {
        drive.setMotorsBreakMode();
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsFloatMode() {
        drive.setMotorsFloatMode();
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void slideMotorsBrakeMode() {
        bigGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motors", "Brake Mode");
    }
    public void slideMotorsFloatMode() {
        bigGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        smallGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Motors", "Float Mode");
    }

}

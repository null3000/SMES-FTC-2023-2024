/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Teleop", group="Linear OpMode")
public class Teleop extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor linSlide = null;

    @Override
    public void runOpMode() {


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");


        linSlide = hardwareMap.get(DcMotor.class, "lin_slide");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        linSlide.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MoveRobot();
            MoveSlide();


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }
    }

    public void MoveRobot(){ // tested and working
        double max;

        double axial   = -gamepad1.left_stick_y;  // pushing stick forward gives negative value, for some stupid reason
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

//       complicated math stuff people smarter than me did:
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // clean the values so no wheel power exceeds 100%, or else would be bad
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

//        speed modes
        double slowModeMultiplier = 0.33;
        double mediumModeMultiplier = 0.66;

        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
            leftFrontPower  *= 1;
            rightFrontPower *= 1;
            leftBackPower   *= 1;
            rightBackPower  *= 1;
        } else if(gamepad1.right_bumper) {
            leftFrontPower  *= mediumModeMultiplier;
            rightFrontPower *= mediumModeMultiplier;
            leftBackPower   *= mediumModeMultiplier;
            rightBackPower  *= mediumModeMultiplier;
        } else {
            leftFrontPower  *= slowModeMultiplier;
            rightFrontPower *= slowModeMultiplier;
            leftBackPower   *= slowModeMultiplier;
            rightBackPower  *= slowModeMultiplier;
        }
//      test code, uncomment when using

//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

    public void MoveSlide(){ //        this code hasn't been tested yet, so it might not work


        double SlidePower = 0;

//        by default, slide motor is not moving
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            when a is pressed, slide motor goes up
        if (gamepad1.a) {
            SlidePower = 1;
        }
//            when b is pressed, slide motor goes down
        else if (gamepad1.b) {
            SlidePower = -1;
        }
        ;
        double currentSlidePower = linSlide.getPower();
        double slidePowerIncrement = 0.1;
        if (currentSlidePower < SlidePower) {
            linSlide.setPower(currentSlidePower + slidePowerIncrement);
        } else if (currentSlidePower > SlidePower) {
            linSlide.setPower(currentSlidePower - slidePowerIncrement);
        }




        // Show the elapsed game time and wheel power.
        telemetry.addData("Power", "Slide Power: " + SlidePower);
        telemetry.update();
    }


}

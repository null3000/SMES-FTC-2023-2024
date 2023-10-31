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

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Encoder Linear Slide Test", group = "Linear Opmode")
public class EncLinearSlideTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // MAX_TICKS is the value at the top (don't raise up more than this)
    // MIN_TICKS is the value at the bottom (don't wind up more than this)
    final int MAX_TICKS = 1500;
    final int MIN_TICKS = 0;

    private DcMotorEx linearSlide;

    boolean upperBoundHit = false;

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
            controlLinearSlide();
            telemetry.addData("Slide encoder value: ", linearSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initializeHardware() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        linearSlide = hardwareMap.get(DcMotorEx.class, "slide");

        // Ensuring the motors get the instructions
        sleep(100);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void controlLinearSlide() {

        if (gamepad2.b) {
            // Going up

            // If the linear slide has hit the top then the upperBoundHit becomes true
            // We can only move the linear slide up if upperBoundHit is false
            // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
            if (linearSlide.getCurrentPosition() >= MAX_TICKS)
                upperBoundHit = true;
            else if (linearSlide.getCurrentPosition() < MAX_TICKS - 100)
                upperBoundHit = false;

            // If the current position is valid, we move the motor upwards
            // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
            if ((linearSlide.getCurrentPosition() < MAX_TICKS) && (!upperBoundHit))
                linearSlide.setPower(1);
            else
                linearSlide.setPower(0);

        } else if (gamepad2.a) {
            // Going down

            // If the current position is valid, we move the motor upwards
            // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
            if ((linearSlide.getCurrentPosition() > MIN_TICKS))
                linearSlide.setPower(-.5);
            else
                linearSlide.setPower(0);
        } else {
            linearSlide.setPower(0);
        }


//        if (gamepad2.b)
//            if (linearSlide.getCurrentPosition() >= MAX_TICKS) {
//                linearSlide.setPower(0);
//            } else {
//                linearSlide.setPower(1);
//            }
//        else if (gamepad2.a)
//            if (linearSlide.getCurrentPosition() <= MIN_TICKS) {
//                linearSlide.setPower(0);
//            } else {
//                linearSlide.setPower(-.5);
//            }
//        else
//            linearSlide.setPower(0);
    }
}

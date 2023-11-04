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

package org.firstinspires.ftc.teamcode.templates;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.driveTools.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.driveTools.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.teleOp_Constants;

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


//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Normal Drive Mode", group = "Linear Opmode")
@Disabled
public class Template_TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Motors + Encoders
    SampleMecanumDrive drive = null;
    StandardTrackingWheelLocalizer encoders = null;

    // Extra hardware
    private DcMotor vertLinearSlide = null;
    private Servo claw = null;

    // All drive constants are stored inside the c object
    public teleOp_Constants c = null;


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

        // Loading Drive Constants
        double maxSpeed = c.getMaxSpeed();


        // Inputs for directional motor power
        float x = gp.left_stick_x;
        float y = -gp.left_stick_y;

        // This gives the output power by interpreting the joystick data (the first parameter).
        // The last two parameters limit the output power to the maxSpeed of the robot specified in teleOp_Constants
        // This only calculates the power to the motors before rotation, see the next set of Range.clips for rotation.
        double lfp = Range.clip(x + y, -maxSpeed, maxSpeed);
        double lbp = Range.clip(y - x, -maxSpeed, maxSpeed);
        double rfp = Range.clip(y - x, -maxSpeed, maxSpeed);
        double rbp = Range.clip(x + y, -maxSpeed, maxSpeed);


        // Input for rotational motor power
        float rot = -gp.right_stick_x;

        // This sets the motors output power by adding/subtracting rotational values from the motors.
        // The reason the min/max is set to ±1.0 is because it is ok for this calculation to exceed the max motor power
        // If two of the motors exceed the max speed for this calculation, the other two will cancel it out,
        // causing the robot to still move at the speed provided by the above Range.clip calculations
        // Essentially the robot can't actually move faster than the maxSpeed variable even though the calculation isn't clipped by maxSpeed
        lfp = Range.clip(lfp - rot, -1.0, 1.0);
        lbp = Range.clip(lbp - rot, -1.0, 1.0);
        rfp = Range.clip(rfp + rot, -1.0, 1.0);
        rbp = Range.clip(rbp + rot, -1.0, 1.0);

        // This allows slow mode to work, allowing the drivers to precisely control the robot.

        double speedModifier = 1;
        if (gp.right_bumper) {
            speedModifier = .4;
            drive.setMotorsBreakMode();
        } else
            drive.setMotorsFloatMode();

        // speedModifier has a domain of [0, 1]
        lfp *= Range.clip(speedModifier, 0.0, 1.0);
        lbp *= Range.clip(speedModifier, 0.0, 1.0);
        rbp *= Range.clip(speedModifier, 0.0, 1.0);
        rfp *= Range.clip(speedModifier, 0.0, 1.0);

        // This method sends the power we've calculated to the wheels
        // There should only be ONE of these commands ANYWHERE in this class.
        drive.setMotorPowers(lfp, lbp, rbp, rfp);

        // This is everything that will print to the console on the Drive Hub. Feel free to add/remove as you like
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Power", "lf (%.2f), rb (%.2f), lb (%.2f), rf (%.2f)", lfp, rbp, lbp, rfp);
        telemetry.addData("Speed Modifier", "(%.2f)", speedModifier);
        telemetry.addData("Slide encoder value: ", vertLinearSlide.getCurrentPosition());
        telemetry.addData("Slide power", vertLinearSlide.getPower());
        telemetry.addData("Left Encoder", encoders.getWheelPositions().get(0));
        telemetry.addData("Right Encoder", encoders.getWheelPositions().get(1));
        telemetry.addData("Front Encoder", encoders.getWheelPositions().get(2));
        telemetry.update();
    }

    public void initializeHardware() {

        // Initialize the hardware variables.
        // Make sure the deviceName strings are the same here as they are in the driver hub

        // The hardwareMap is defined inside of SampleMecanumDrive for consistency between opModes
        drive = new SampleMecanumDrive(hardwareMap);

        // Same thing for encoders here
        encoders = new StandardTrackingWheelLocalizer(hardwareMap);

        // Extra hardware
        vertLinearSlide = hardwareMap.get(DcMotor.class, "vertSlide");
        claw = hardwareMap.get(Servo.class, "claw");

        // vert slide init
        vertLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // This makes sure the motors are not using encoders (we don't use them)
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensuring the motors get the instructions
        sleep(100);

        // Make sure the robot doesn't roll around while initialized
        drive.setMotorsBreakMode();
    }

    private void controlLinearSlide(Gamepad gp) {


    }

    public void controlClaw(Gamepad gp) {


        if (gp.b)
            claw.setPosition(c.getClawClose());
        else if (gp.x)
            claw.setPosition(c.getClawOpen());
    }

}

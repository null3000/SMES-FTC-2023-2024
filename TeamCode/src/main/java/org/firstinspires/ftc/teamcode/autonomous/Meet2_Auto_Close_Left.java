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

package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.AutoData.encoderInchesToTicks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.driveTools.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.driveTools.StandardTrackingWheelLocalizer;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

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

//drive straight forward
// (line up robot touching back wall at an angle so that driving straight forward corresponds with tallest pole)
// then, lift arm, drop off pre-load cone
@Autonomous
public class Meet2_Auto_Close_Left extends LinearOpMode {

    SampleMecanumDrive drive = null;
    StandardTrackingWheelLocalizer encoders = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Slide and Claw Objects
    private DcMotor vertLinearSlide = null;

    // Linear Slide
    private DcMotor smallGear = null;
    private DcMotor bigGear = null;

    private Servo Servo1 = null;

    private Servo Servo2 = null;
    private int autoPhase = 0;

    //camera and CV
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //tags from tag family thing
    int PARK_LEFT = 3;
    int PARK_MIDDLE = 15;
    int PARK_RIGHT = 17;

    //other cv stuff
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        // Method to assign and initialize the hardware
        initializeHardware();
        drive.setMotorsBreakMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // First Move
        //all in inches btw

        double initialForward = 3;
        double strafeLeft = 28;

        Trajectory initPush = drive.trajectoryBuilder(new Pose2d())
                .forward(initialForward)
                .build();


        Trajectory strafeToLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(strafeLeft)
                .build();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            /*
            Our Autonomous is broken into phases.
            This is so that we don't have the motors receiving contradictory commands.
            Once one phase of instructions is complete, we will move to the next.
             */


            if (autoPhase == 0) {

                Servo2.setPosition(.55);
                Servo1.setPosition(0.19); // Init to close
                sleep(2000);
                Servo2.setPosition(.9);
                sleep(2000);

                drive.followTrajectory(initPush);
                drive.followTrajectory(strafeToLeft);

                autoPhase = 1;
            }
        }
    }


    private int checkConeState() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wbcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//        int retryCount = 0;
//        while ((currentDetections == null || currentDetections.size() == 0) && retryCount < 5 && opModeIsActive()) {
//            currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            retryCount++;
//            sleep(100);
//        }
//        if (retryCount == 5) {
//            telemetry.addLine("couldn't detect tag");
//            telemetry.update();
//            return 1;
//        }
        //if it can't detect the tag in 10 detection cycles, i define this as unable to detect the tag
        //this means itll just go to the middle square
        while (currentDetections.size() < 5 && opModeIsActive()) {
            currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections != null) {
                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == PARK_LEFT) {
                            tagOfInterest = tag;
                            camera.closeCameraDevice();
                            return 0;
                        }
                        if (tag.id == PARK_MIDDLE || tag == null) {
                            tagOfInterest = tag;
                            camera.closeCameraDevice();
                            return 1;
                        }
                        if (tag.id == PARK_RIGHT) {
                            tagOfInterest = tag;
                            camera.closeCameraDevice();
                            return 2;
                        }
                    }
                }
                sleep(100);
            }
        }
        camera.closeCameraDevice();
        return 1;
    }

    private boolean moveLift(int targetTicks) {

        // Moving the lift to the target position

        // Constants
        double liftTicks = vertLinearSlide.getCurrentPosition();
        final double LIFT_IDLE_POWER = 0.3;
        final double LIFT_MIN_POWER = -0.3;
        final double LIFT_MAX_POWER = 1.0;


        // Checking to see if the lift is at the position with a tolerance of ±50 ticks
        if (Math.abs(liftTicks - targetTicks) < 50) {
            vertLinearSlide.setPower(LIFT_IDLE_POWER);
            return true;
        }


        double distToTarget = -(liftTicks - targetTicks);

        double calcPower = Range.clip(distToTarget / 1000, LIFT_MIN_POWER, LIFT_MAX_POWER);

        vertLinearSlide.setPower(calcPower);

        return false;
    }


    public boolean move(double inches, int dir) {

        /* DIR KEY
        0 = forward
        1 = right
        2 = back
        3 = left
         */

        // We measure encoder distance based on the average of the left and right encoder
        double encoderPos = (encoders.getWheelPositions().get(0) + encoders.getWheelPositions().get(1)) / 2;
        double ticks = encoderInchesToTicks(inches);

        // Moving forward if we are not at the destination
        if (encoderPos < ticks)
            move(dir);
        else
            return true;
        return false;
    }

    private void move(int dir) {

        /*
        0 = forward
        1 = right
        2 = back
        3 = left
         */

        int x;
        int y;


        switch (dir) {

            case 0:
                y = -1;
                x = 0;
                break;
            case 1:
                y = 0;
                x = -1;
                break;
            case 2:
                y = 1;
                x = 0;
                break;
            case 3:
                y = 0;
                x = 1;
                break;
            default:
                x = 0;
                y = 0;
                break;
        }

        if (opModeIsActive()) {
            double lfp = (Range.clip(x + y, -1.0, 1.0));
            double lbp = (Range.clip(y - x, -1.0, 1.0));
            double rfp = (Range.clip(y - x, -1.0, 1.0));
            double rbp = (Range.clip(x + y, -1.0, 1.0));

            drive.setMotorPowers(lfp, lbp, rfp, rbp);

        }
    }


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }




// Old stuff

//            //close claw to tighten around pre-load cone
//            claw.setPosition(1);
//            //give the claw closing time before jerking forward
//            sleep(100);
//            leftFrontDrive.setPower(1);
//            leftBackDrive.setPower(1);
//            rightBackDrive.setPower(1);
//            rightFrontDrive.setPower(1);
//            // once this loop completes, the slide should be at the top
//            // (yes, the cone is dangling in the air)
//            // yes, it has a risk of falling
//            // however, our lack of encoder/time/autonmous in general accuracy,
//            // the less driving adjustments we have to make the better
//            while (linearSlide.getCurrentPosition() <= MAX_SLIDE_TICKS) {
//                linearSlide.setPower(-1);
//            }
//            // COMMENT OUT NOT-USED OPTIONS
//            //the time to be driving forward if not using encoders:
//            sleep(500);
//            // if we are using encoders:
//            if (STOP_DIST_TICKS >= encoder.getCurrentPosition()) {
//                leftFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                // resetting to 0 so that we can back up
//                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            // drop the cone
//            claw.setPosition(0.7);
//            // back-up
//            leftFrontDrive.setPower(-1);
//            leftBackDrive.setPower(-1);
//            rightBackDrive.setPower(-1);
//            rightFrontDrive.setPower(-1);
//            // TIME
//            sleep(500);
//            // OR TICKS
//            if (BACK_UP_TICKS >= encoder.getCurrentPosition()) {
//                leftFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                // resetting to 0 so that we can back up
//                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            //put the slide down after backing up
//            while (linearSlide.getCurrentPosition() <= MAX_SLIDE_TICKS) {
//                linearSlide.setPower(1);
//            }

//    private void raiseLift(int ticks) {
//
//        // Going up smoothly
//
//        // If the linear slide has hit the top then the upperBoundHit becomes true
//        // We can only move the linear slide up if upperBoundHit is false
//        // upperBoundHit becomes false again when we have left the buffer threshold (100 ticks) below the top
//        if (vertLinearSlide.getCurrentPosition() >= ticks)
//            upperBoundHit = true;
//        else if (vertLinearSlide.getCurrentPosition() < ticks - 100)
//            upperBoundHit = false;
//
//        // If the current position is valid, we move the motor upwards
//        // The second conditional is to make sure the motor doesn't go clank clank at the top (basically a buffer)
//        if ((vertLinearSlide.getCurrentPosition() < ticks - 150) && (!upperBoundHit))
//            vertLinearSlide.setPower(1);
//        else if ((vertLinearSlide.getCurrentPosition() < ticks && (!upperBoundHit)))
//            vertLinearSlide.setPower(0.4);
//        else if (vertLinearSlide.getCurrentPosition() > 100)
//            // To prevent downward drift
//            vertLinearSlide.setPower(0.3);
//
//    }

    public void initializeHardware() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive = new SampleMecanumDrive(hardwareMap);
        Servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");


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

    public void setMotorsBreakMode() {
        drive.setMotorsBreakMode();
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
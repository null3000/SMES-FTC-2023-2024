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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.driveTools.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.driveTools.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

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
public class GoodAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    SampleMecanumDrive drive = null;
    StandardTrackingWheelLocalizer encoders = null;
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "tpModel.tflite";
    private static final String[] LABELS = {
            "blueTp"
    };


    // Linear Slide
    private DcMotor smallGear = null;
    private DcMotor bigGear = null;

    private Servo Servo1 = null;

    private Servo Servo2 = null;
    private int autoPhase = 0;

    private int leftSpikeThreshold = 150;
    private int centerSpikeThreshold = 151;

    private String scenario = "";
    @Override
    public void runOpMode() {



        // Method to assign and initialize the hardware
        initializeHardware();
        initTfod();
        drive.setMotorsBreakMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


//        waits for camera to find object
        sleep(3000);


        List<Recognition> currentRecognitions = tfod.getRecognitions();
//            get the recognition with the highest confidence
        Recognition teamProp = null;
        for (Recognition recognition : currentRecognitions) {
            if (teamProp == null || recognition.getConfidence() > teamProp.getConfidence()) {
                teamProp = recognition;
            }
        }
        if (teamProp == null) {
            scenario = "right";
        } else {
            double x = (teamProp.getLeft() + teamProp.getRight()) / 2;
            if (x < leftSpikeThreshold) {
                scenario = "left";
            } else if (x > centerSpikeThreshold){
                scenario = "center";
            } else {
                scenario = "right";
            }
        }

        // First Move
        // in inches



        Trajectory initPush = drive.trajectoryBuilder(new Pose2d())
                .forward(23)
                .build();


        Trajectory strafeToRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(12)
                .build();

        Trajectory strafeToLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(14)
                .build();

        Trajectory backUp = drive.trajectoryBuilder(new Pose2d())
                .back(12)
                .build();
        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(6)
                .build();


        telemetry.addData("scenario", scenario);
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetryTfod();
            telemetry.update();


//            sleep to not take all of CPU
            sleep(20);

            /*
            Our Autonomous is broken into phases.
            This is so that we don't have the motors receiving contradictory commands.
            Once one phase of instructions is complete, we will move to the next.
             */


//            auto phase 0 is to make sure clamp the pixel then move to the center between the 3 spike marks
            if (autoPhase == 0) {
                Servo1.setPosition(0.19); // Init to close
                sleep(2000);
                Servo2.setPosition(.9);
                sleep(2000);
                drive.followTrajectory(initPush);
                autoPhase = 1;
            }

//            auto phase 1 is to move to the correct spike mark based on the scenario

            if (autoPhase == 1) {
                sleep(2000);
                if (scenario.equals("left")) {
                    drive.followTrajectory(strafeToLeft);
                } else if (scenario.equals("center")) {
                    drive.followTrajectory(forward);
                } else if (scenario.equals("right")) {
                    drive.followTrajectory(strafeToRight);
                }
                autoPhase = 2;
            }
        }
    }

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


    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

}
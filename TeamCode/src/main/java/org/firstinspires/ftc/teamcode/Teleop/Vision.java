package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


            AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();

            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640,480))
                    .build();

        waitForStart();

        while (opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0){

                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("range", tag.ftcPose.range);
            }

            telemetry.update();
        }









    }
}

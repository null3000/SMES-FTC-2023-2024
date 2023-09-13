package org.firstinspires.ftc.teamcode.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@TeleOp
public class WebcamTest extends LinearOpMode {
    int parkSpot;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wbcam"), cameraMonitorViewId);
        webcam.setPipeline(new Pipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */

            if(gamepad1.a)
            {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            sleep(100);
        }
    }

    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {

            //TODO: WHITE BALANCING
            //this algorithm was largely inspired by GIMP'S algorithm for white balancing
            //https://docs.gimp.org/2.10/en/gimp-layer-white-balance.html
            //basically, the end values of each individual r g b layer are removed
            //for example, if a red layer covers a full spectrum of red values (so 0-255)
            //the output may cut down this spectrum to 10-240
            //then these new values are mapped back to 0-255, so some layers will be missing values (this is intended)


            int width = input.width(); //width of image
            int height = input.height(); //height of image

            List<List<Integer>> imageList = new ArrayList<List<Integer>>();
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    double[] data = input.get(i,j);

                    int R = (int) data[0];
                    int G = (int) data[1];
                    int B = (int) data[2];

                    imageList.add(Arrays.asList(j, i, R, G, B));
                }
            }

            imageList = balanceImageArray(imageList);

            int offset = 10;

            //counts instances of what may be a cone spot, the largest one will
            int [] parkArray = {0,0,0};

            for (int i = 0; i < imageList.size(); i++) {
                int currRed = imageList.get(i).get(2);
                int currGreen = imageList.get(i).get(3);
                int currBlue = imageList.get(i).get(4);

                int fwdRed = imageList.get(i+3).get(2);
                int fwdGreen = imageList.get(i+3).get(3);
                int fwdBlue = imageList.get(i+3).get(4);
                if ((currRed >= 255 - offset && currGreen <= offset && currBlue <= offset) && (fwdRed <= offset && fwdGreen <= offset && fwdBlue >= 255 - offset)) {
                    parkArray[0] += 1;
                }

                //if current is mostly white and forward is mostly green, it could be the 2nd spot
                if ((currRed >= 255 - offset && currGreen >= 255 - offset && currBlue >= 255 - offset) && (fwdRed <= offset && fwdGreen >= 255 && fwdBlue <= offset)) {
                    parkArray[1] += 1;
                }

                //if current is mostly black and forward is mostly yellow, it could be the 3rd spot
                if ((currRed <= offset && currGreen <= offset && currBlue <= offset) && (fwdRed >= 255 - offset&& fwdGreen >= 255 - offset && fwdBlue <= offset)) {
                    parkArray[2] += 1;
                }
            }


            //the largest array value will be where to park
            if (parkArray[0] >= parkArray[1] && parkArray[0] >= parkArray[2]) {
                parkSpot = 1;
            }
            if (parkArray[1] >= parkArray[0] && parkArray[1] >= parkArray[2]) {
                parkSpot = 2;
            }
            if (parkArray[2] >= parkArray[0] && parkArray[2] >= parkArray[1]) {
                parkSpot = 3;
            }

            telemetry.addData("park spot", parkSpot);
            telemetry.update();
            return input;
        }
    }
    public static List<List<Integer>> balanceImageArray( List<List<Integer>> imageList) {
        double percent = .05;
        double clip = percent*imageList.size();
        clip = (int) clip;

        for (int c = 2; c < 5; c++) { //c refers to the channel, so 2 = red 3 = green or 4 = blue values
            int newLow = 0;
            int newHigh = 0;

            int amtClipped = 0;
            int start = 255;
            while (amtClipped < clip) {
                for (int i = 0; i < imageList.size(); i++) {
                    if (imageList.get(i).get(c) == start) {
                        amtClipped++;
                    }
                }
                //decrements start val by 1
                start--;
            }
            newHigh = start;

            //comment
            start = 0;
            amtClipped = 0;
            while (amtClipped < clip) {
                for (int i = 0; i < imageList.size(); i++) {
                    if (imageList.get(i).get(c) == start) {
                        amtClipped++;
                    }
                }
                //increments start val by 1 to remove more pixels
                start++;
            }
            newLow = start;
            System.out.println(newLow);

            //removes pixel vals at the highest ends of the r g b spectrum
            System.out.println(newHigh);
            //stretches the channel to fit the full rgb spectrum
            //for example, say the new highest red value is 247, this turns 247 into 255 and so on
            for (int i = 0; i < imageList.size(); i++) {
                if (imageList.get(i).get(c) > newHigh) {
                    imageList.get(i).set(c,newHigh);
                }
                if (imageList.get(i).get(c) < newLow) {
                    imageList.get(i).set(c, newLow);
                }
            }

            for (int i = 0; i < imageList.size(); i++) {
                if (imageList.get(i).get(c) != 0) {
                    imageList.get(i).set(c, map_val(imageList.get(i).get(c), newLow, newHigh, 0, 255));
                }
            }
        }
        return imageList;
    }

    public static int map_val(int s, int a1, int a2, int b1, int b2) {
        return  (b1 + ((s - a1) * (b2 - b1) / (a2 - a1)));
    }
}
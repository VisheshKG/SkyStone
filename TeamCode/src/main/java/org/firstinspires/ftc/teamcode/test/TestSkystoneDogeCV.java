package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.recognition.SkystoneDetectorDogeCV;

import java.util.ArrayList;
import java.util.Locale;


/*
 * This class is adaptation of SkystoneDetectorExample at https://github.com/FTC9794/SkystoneDogeCVTutorial
 *
 * Thanks to EasyOpenCV for the great API (and most of the example)
 *
 * Original Work Copright(c) 2019 OpenFTC Team
 * Derived Work Copyright(c) 2019 DogeDevs
 */
@TeleOp(name = "Test Skystone Detector", group="Test")

public class TestSkystoneDogeCV extends LinearOpMode {

    private static final int IMG_WIDTH = 640;
    private static final int IMG_HEIGHT = 360;
    private static final int NUM_STONES = 6;
    private static final int STONE_RECT_AREA_MIN = 4000;
    private static final int STONE_RECT_AREA_MAX = 10000;

    private OpenCvCamera phoneCam;
    private int previousCount = 0;
    private SkystoneDetectorDogeCV skystoneDetector;
//    private StoneDetector stoneDetector;

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skystoneDetector = new SkystoneDetectorDogeCV();
        skystoneDetector.setTargetAreaSize(STONE_RECT_AREA_MIN,STONE_RECT_AREA_MAX);
        phoneCam.setPipeline(skystoneDetector);

        // OR StoneDetector if we are interested in Stone only (not interested in Skystone)
        //stoneDetector = new StoneDetector();
        //phoneCam.setPipeline(stoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(IMG_WIDTH, IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        Rect rect;
        while (opModeIsActive())
        {
            // if we have already analyzed the current picture frame, then simply return
            int currentCount = skystoneDetector.getRectangleDetectionCount();
            if (currentCount == previousCount) {
                continue;
            }
            telemetry.addData("Detection Count", "Current=%d, Previous=%d", currentCount, previousCount);

            calculateSkystoneLocation();

            previousCount = currentCount;

            // CAUTION: NOTE that Y-coordinate is printed before X-coordinate deliberately
            // Phone camera rotation is locked to Portrait UPRIGHT however
            // the phone itself is oriented landscape (sideways) left side is phone top
            // Y-coordinate is horizontal along the floor, increasing left to right
            // X-coordinate is vertical, increasing from bottom to top away from the floor

            if (skystoneDetector.isDetected()) {
                rect = skystoneDetector.foundRectangle();
                telemetry.addData("Best Rectangle", "{y=%03d, x=%03d, %03dx%03d} area=%.0f", rect.y, rect.x, rect.width, rect.height, rect.area());
                telemetry.addData("Stone Position X", skystoneDetector.getRectanglePosition().x);
                telemetry.addData("Stone Position Y", skystoneDetector.getRectanglePosition().y);
            }
            ArrayList<Rect> skystoneRects = skystoneDetector.getSkystoneCandidates();
            for (int i = 0; i < skystoneRects.size(); i++) {
                rect = skystoneRects.get(i);
                telemetry.addData("SkyStone " + i, "{y=%03d, x=%03d, %03dx%03d} area=%.0f", rect.y, rect.x, rect.width, rect.height, rect.area());
            }
            ArrayList<Rect> stoneRects = skystoneDetector.getStoneCandidates();
            for (int j = 0; j < stoneRects.size(); j++) {
                rect = stoneRects.get(j);
                telemetry.addData("Stone " + j, "{y=%03d, x=%03d, %03dx%03d} area=%.0f", rect.y, rect.x, rect.width, rect.height, rect.area());
            }

            // Display other image pipeline statistics if needed for debugging
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
//            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();


            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x) {
                phoneCam.pauseViewport();
            }
            else if(gamepad1.y) {
                phoneCam.resumeViewport();
            }
        }
    }

    int quarry[] = new int[NUM_STONES+1];
    int count1, count2, count3;


    protected void calculateSkystoneLocation() {

        ArrayList<Rect> skystoneRects = skystoneDetector.getSkystoneCandidates();
        for (Rect rect : skystoneRects) {
            // skystone height / width ratio must be within a range around 1.6
            double w = rect.width;
            double h = rect.height;
            double ratio = Math.max(Math.abs(h / w), Math.abs(w / h)); // We use max in case h and w get swapped due to image rotation
            if (ratio < 1.4 || ratio > 1.8) {
                count1++;
                continue;
            }
            // the bottom of rectangle (x value) should be close to 66% of the image dimension
            if (Math.abs(rect.x - (IMG_HEIGHT * 0.66)) > 40) {
                count2++;
                continue;
            }
            // the left side of rectangle (y value) cannot start less than 160 out of 640 pixels
            int leftlimit = IMG_WIDTH / 4;
            if (rect.y < (leftlimit)) {
                count3++;
                continue;
            }
            // if we still have a rectangle then it meets all requirements of skystone in quarry
            // calculate position (value = 0, 1, 2 or 3)
            int pos = (rect.y - leftlimit) * 4 / (IMG_WIDTH - leftlimit);
            // stone position in the quarry is counted from the audience wall, reverse the count
            pos = NUM_STONES - pos;

            // skystone detected at position pos in the quarry, increment count for that position
            quarry[pos]++;
        }
        telemetry.addData("Quarry", "6:[%d  %d  %d  %d  %d  %d]:1", quarry[6], quarry[5], quarry[4], quarry[3], quarry[2], quarry[1]);
        telemetry.addData("Rejected", "ratio=%d, distance=%d, pan=%d", count1, count2, count3);
    }
}

package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.recognition.SkystoneDetectorDogeCV;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.purepursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;
import org.firstinspires.ftc.teamcode.skystone.FieldSkystone.AllianceColor;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Each floor tile is 23.5 inch square (counting tabs on one side and not on the other side)
 * Each floor tile with all side tabs cut off is 22.75 inch square
 * The tabs add 0.75 to tile width on each side.
 * Field width = 23.5 * 6 - 0.75 = 70.25 each side square
 *
 * Robot is 18x18 square. Robot (x,y) position is at the center of the robot.
 */

public abstract class SkystoneAutoBase extends LinearOpMode {

    // constants
    private static final int IMG_WIDTH = 640;
    private static final int IMG_HEIGHT = 360;
    private static final int STONE_RECT_AREA_MIN = 4000;
    private static final int STONE_RECT_AREA_MAX = 10000;

    // OpMode members here
    protected MecaBot robot;
    protected MecaBotMove nav;
    protected OdometryGlobalPosition globalPosition;
    private OpenCvCamera phoneCam;
    private SkystoneDetectorDogeCV skystoneDetector;
//    private StoneDetector stoneDetector;

    protected AllianceColor aColor;
    protected String actionString = "Inactive";
    protected String message = "NO";
    private int previousCount = 0;
    boolean haveSkystone = false;

    /*
     * Abstract methods, must be implemented by the sub-classes
     */
    public abstract void setOdometryStartingPosition();

    public abstract String getColorString();

    public ColorSensor chooseColorSensorForSkystone() {
        // random choice, the RED and BLUE subclasses should override this method
        return robot.leftColorSensor;
    }

    public String getAction() {
        return actionString;
    }

    public String getMessage() {
        return message;
    }

    protected double flipX4Red(double value) {
        return (aColor == AllianceColor.BLUE) ? value : -value;
    }

    protected double flipAngle4Red(double value) {
        if (aColor == AllianceColor.RED) {
            value = MathFunctions.angleWrap(value + 180);
        }
        return value;
    }

    /**
     * Initialize all hardware and software data structures
     */
    public void initializeOpMode() {

        // Initialize the robot hardware and drive system variables.
        robot = new MecaBot();
        robot.init(hardwareMap);
        // disabled: lof of problems with gyro rotation crashing the problem
        // robot.initIMU();
        // initialize the OpenCV image recognition for skystone detection
        initSkystoneDetectionOpenCV();
        // initialize movement algorithms variables
        nav = new MecaBotMove(this, robot);
        // odometry is initialize inside drive system MecaBotMove class
        globalPosition = nav.getPosition();
        // this method is overridden by sub-classes to set starting coordinates for RED/BLUE side of field
        setOdometryStartingPosition();
        // start the thread to calculate robot position continuously
        nav.startOdometry();
        // start printing messages to driver station asap
        setupTelemetry();
    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        while (opModeIsActive()) {

            telemetry.update();
        }
    }

    /**
     * do everything in autonomous mode
     * detect a skystone, pick it up, transport and delivery to the foundation,
     * move the foundation, go park itself under the skybridge
     */
    public void runFullAutoProgram() {

// disabled: color sensor detection does not allow robot movement accuracy
// replaced with camera detection, using image recognition
//        positionToDetectSkystoneWithColorSensor();
//        pickupSkystoneWithColorSensor();

        printSkystoneDetection(1.0);
        int pos = skystoneDetector.getSkystoneLocationInQuarry();
        pickupSkystoneAtPosition(pos);
        deliverSkystone();
        moveFoundation();
        parkAtInsideLane();
    }

    protected void setupTelemetry() {

        actionString = "Telemetry";
        telemetry.addLine("Auto ")
                .addData(getColorString(), new Func<String>() {
                    @Override
                    public String value() {
                        return getAction();
                    }
                })
                .addData("msg", new Func<String>() {
                    @Override
                    public String value() {
                        return getMessage();
                    }
                });
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return nav.getMovementStatus();
                    }
                });
        telemetry.addLine("Position ")
                .addData("X", "%3.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Angle", "%4.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                })
                .addData("F", new Func<String>() {
                    public String value() {
                        return robot.getFrontDirection();
                    }
                });
        message = "Done";
        telemetry.update();
    }
    protected void initSkystoneDetectionOpenCV() {

        // Instantiate an OpenCvCamera object for the camera we'll be using.
        //we're using the phone's internal camera and selected the BACK camera.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        // Open the connection to the camera device
        phoneCam.openCameraDevice();

        // Specify the image processing pipeline we wish to invoke upon receipt
        // of a frame from the camera. Note that switching pipelines on-the-fly
        // (while a streaming session is in flight) *IS* supported.
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

    }

    /**
     * printSkystoneDetection()
     * This method prints all the internal variables using OpenCV for image recognition and
     * calculating the best position of the skystone in the quarry. Useful for development.
     * In final program this method is not used, when the skystone detection is working reliably.
     */
    public void printSkystoneDetection(double timeout) {

        actionString = "Skystone Detection";
        message = "Detecting";
        ElapsedTime runTime = new ElapsedTime();
        while (opModeIsActive() && runTime.seconds() < timeout) {
            // if we have already analyzed the current picture frame, then simply return
            int currentCount = skystoneDetector.getRectangleDetectionCount();
            if (currentCount == previousCount) {
                sleep(50);
                continue;
            }
            telemetry.addData("Detection Count", "Current=%d, Previous=%d", currentCount, previousCount);

            int[] quarry = skystoneDetector.getStoneQuarryCounts();
            telemetry.addData("Quarry", "6:[%d  %d  %d  %d  %d  %d]:1", quarry[6], quarry[5], quarry[4], quarry[3], quarry[2], quarry[1]);
            int pos = skystoneDetector.getSkystoneLocationInQuarry();
            message = "Location " + pos;
            telemetry.addData("Skystone Detected", "%d  %d  %d  %d  %d  %d  %d  %d  %d  %d", pos, pos, pos, pos, pos, pos, pos, pos, pos, pos);
            int[] reject = skystoneDetector.getRejectCounts();
            telemetry.addData("Rejected", "ratio=%d, distance=%d, pan=%d", reject[1], reject[2], reject[3]);

            Rect rect;
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
            telemetry.addData("Image Size", "w=%.0f x h=%.0f", skystoneDetector.getSize().width, skystoneDetector.getSize().height);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.update();
            previousCount = currentCount;
        }
    }

    /**
     * Pickup skystone at known position number, which is determined by camera recognition
     *
     * @param pos   Skystone position number in the quarry. Position #1 is adjacent to wall.
     *              Position #6 is near the skybridge in center of field.
     */
    public void pickupSkystoneAtPosition(int pos) {

        actionString = "Skystone Pickup";
        message = "Location" + pos;

        // Starting position is (green wheels facing the center of the field, +ve Y-Axis)
        // this must be set in the setOdometryStartingPosition() method
        //globalPosition.initGlobalPosition(flipX4Red(+33.0), +8.5, 90.0);

        robot.setFrontIntake();

        // stone quarry is 47 inches from the BLUE/RED wall, 48 inches from the audience wall
        // move forwards towards the stone quarry corresponding to skystone position number
        // (assumption: skystone position has been detected by camera visual recognition)
        double xpos = FieldSkystone.HALF_LENGTH - (pos * FieldSkystone.STONE_LENGTH + 8);
        double ypos = FieldSkystone.TILE_LENGTH * 1.5;  // middle of 2nd tile
        nav.goToPosition(flipX4Red(xpos), ypos);

        // Rotate to a diagonal heading so one green wheel will clear skystone and wrap around
        message = String.format("rotate (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        nav.odometryRotateToHeading(flipAngle4Red(48.0));
        // start the intake green wheels for stone pickup
        robot.runIntake(0.6);

        // Move the robot diagonal to position intake in front of skystone
        message = String.format("diagonal (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        nav.odometryMoveDistance(flipX4Red(-6.0), MecaBotMove.DriveType.DIAGONAL);

        // Turn robot intake around the skystone to pick it up
        message = String.format("rotate (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        nav.odometryRotateToHeading(flipAngle4Red(20));  // no need to turn to zero degrees, the block is picked up before that and we need to go back at an angle

        // Go back to the 2nd tile lane in preparation for run to deliver the skystone
        // This should be enough time to fully move the stone inside the robot, stop the intake wheels
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        robot.setFrontLiftarm();
        nav.goToPosition(flipX4Red(xpos-12), ypos);
        robot.stopIntake();

        ColorSensor cs = robot.blockColorSensor;
        if (isSkystone(cs)) {
            haveSkystone = true;
            robot.grabStoneWithClaw();
            message = "Stone secure with claw";
        }
        else {
            message = "Stone not detected";
        }
    }

    public void positionToDetectSkystoneWithColorSensor() {
        //
        // Staring position is
        // BLUE: globalPosition.initGlobalPosition(14.0, 9.0, 0.0);
        // RED: globalPosition.initGlobalPosition(-14.0, 9.0, 180.0);
        // stone quarry is 47 inches from the BLUE/RED wall, 48 inches from the audience wall
        // move sideways towards the stone quarry (caution we may hit the skybridge side support)
        // movement is towards +ve Y axis LEFT for BLUE , RIGHT for RED
        nav.odometryMoveRightLeft(flipX4Red(-26), MecaBotMove.DRIVE_SPEED_SLOW);

        // ensure robot direction is straight down the stone quarry
        nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS));
/*
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // The gyro initialization CANNOT be controlled by software. It initializes hardware at ZERO angle on program init.
        // GYRO angle is ZERO (+ve X-Axis) towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_SLOW);
 */
        sleep(500);

    }

    public void pickupSkystoneWithColorSensor() {

        boolean found = false;
        // look for skystone in the stone quarry containing 6 stones
        // Notes: the color sensor is mounted 5 inches from the front of the robot

        // choose the correct side color sensor depending on our alliance color
        ColorSensor cs = chooseColorSensorForSkystone();

        // first stone center is at x=26 coordinate location, robot is approximately at x=24 location
        // color sensor is at x=18~20 location, but
        double distanceToNextStone = 5;

        // look for skystones in #1 to #6 position
        for (int i = 1; i <= 6; i++) {
            // move forward one stone length at a time
            nav.odometryMoveForwardBack(distanceToNextStone, MecaBotMove.DRIVE_SPEED_DEFAULT);
            // check for skystone
            if (isSkystone(cs) && i>1) { // skipping skystone in position #1 since picking up runs into skybridge
                found = true;
                // take action to pickup skystone
                telemetry.addData("Detected Skystone at position # ", i);
                telemetry.update();
                break;
            }
            distanceToNextStone = 7.0; // each stone is 8 inches long
        }

        if (!found) {
            // nothing to do really, we can just go to foundation, and pretend to delivery skystone
            return;
        }


        // pick up the skystone
        // move back more than 8 inches to clear the skystone (tuning done)
        nav.odometryMoveForwardBack(-11, MecaBotMove.DRIVE_SPEED_SLOW);
        // move sideways to position green wheels in front of the skystone
        nav.odometryMoveRightLeft(flipX4Red(-13), MecaBotMove.DRIVE_SPEED_DEFAULT);
        // turn intake wheels on for grabbing the skystone
        robot.runIntake(MecaBotMove.DRIVE_SPEED_DEFAULT);
        // move forward to grab the skystone
        nav.odometryMoveForwardBack(6, MecaBotMove.DRIVE_SPEED_MIN);
        // continue to run the intake for a little bit
        sleep(1000);
        // stop the intake, hopefully we have picked the stone already
        robot.stopIntake();
        // move sideways back to the lane under the skybridge
        nav.odometryMoveRightLeft(flipX4Red(+15), MecaBotMove.DRIVE_SPEED_DEFAULT);
        // line up in the lane
        nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS));

        cs = robot.blockColorSensor;
        if (isSkystone(cs)) {
            haveSkystone = true;
            robot.grabStoneWithClaw();
        }
    }

    public boolean isSkystone(ColorSensor cs) {

        float redToblue = ((float) (cs.red() - cs.blue())) / (float) cs.blue();

        telemetry.addData("Blue Reading=", cs.blue());
        telemetry.addData("Red Reading=", cs.red());
        telemetry.addData("(Red - Blue)/blue=", "%.2f", redToblue);
        telemetry.update();

        if (redToblue < 0.20) {
            telemetry.addData("Skystone Found", redToblue);
            return true;
        }
        return false;
    }

    public void deliverSkystone() {
        // let's go to deliver the Skystone
        actionString = "Deliver Skystone";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // Driving in reverse to avoid turn around and crashing into alliance partner robot
        robot.setFrontLiftarm();
        telemetry.update(); // print the new orientation of the robot on driver station
        // destination is the centered on tile in front of center of foundation
        //nav.goToXPosition(flipX4Red(-47), MecaBotMove.DRIVE_SPEED_SLOW);
        nav.goToXPosition(flipX4Red(-9)); // 9 is for indoor test only, full field value = 55
        robot.setFrontIntake();

        message = String.format("rotate (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // turn robot back towards foundation
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_NEG_Y_AXIS);
/*
        nav.gyroRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_NEG_Y_AXIS), MecaBotMove.ROTATE_SPEED_SLOW);
*/
        // practical observation note: Rotation decrements the y-position by 3 inches
        // robot position ~ y=32, robot half length = 8.5, foundation position ~ y=47
        // The above for BLUE field, need to update observation for RED field rotation

        // deliver skystone on the foundation, the lift arm will take time to move, meanwhile we will grab and move foundation.
        if (haveSkystone) {
            nav.moveLiftArmOutside();
        }

        // move backwards to touch the foundation edge
        nav.odometryMoveForwardBack(-8, MecaBotMove.DRIVE_SPEED_SLOW);
        message = String.format("backup (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        telemetry.update(); // print the new orientation of the robot on driver station

    }

    public void moveFoundation() {
        // clamp down on the foundation
        robot.grabFoundation();

        actionString = "Move Foundation";
        message = String.format("turn (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());

        // DO NOT REMOVE this sleep() the clamps take a long time, if we don't sleep the robot moves away before clamping.
        sleep(1000);

        // bring the foundation towards the build zone. When we rotate the foundation in next step,
        // its corner will be in build zone when pushed against the scoreboard wall
        //nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_SLOW);
        //nav.goToPosition(flipX4Red(-38), 18, MecaBotMove.DRIVE_SPEED_DEFAULT, false);
        // trying different approach
        nav.odometryRotateToHeading(flipAngle4Red(-85), MecaBotMove.ROTATE_SPEED_DEFAULT, MecaBotMove.TIMEOUT_SHORT, false);

        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_FAST);

        message = String.format("rotate (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // rotate with the foundation to be square with the walls
        nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS), MecaBotMove.ROTATE_SPEED_FAST, MecaBotMove.TIMEOUT_SHORT, false);
/*
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // The gyro initialization CANNOT be controlled by software. It initializes hardware at ZERO angle on program init.
        // GYRO angle is ZERO towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_DEFAULT);
        //nav.encoderTurn(40, true, MecaBotMove.DRIVE_SPEED_SLOW);
*/
        message = String.format("release (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // enough time elapsed in foundation movement, the lift arm must be out now, release skystone
        if (haveSkystone) {
            robot.releaseStoneWithClaw();
            nav.moveLiftArmInside();
        }
        // drive backwards to push the foundation against the scoreboard wall
        // foundation is 18.5 and half robot is 9
        //disabled
        //nav.encoderMoveForwardBack(-4);

        // foundation has been repositioned, release the clamps
        robot.releaseFoundation();
    }

    public void parkAtInsideLane() {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        // go to middle of a tile in inside lane
        //nav.goToPosition(flipX4Red(-23), 35);
        // straighten up to travel along the X-Axis or the player alliance wall
        //nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS));

        // now go park under the skybridge
        //nav.goToPosition(FieldSkystone.X_ORIGIN, FieldSkystone.TILE_2_CENTER);
        nav.goToPosition(FieldSkystone.TILE_2_CENTER, FieldSkystone.TILE_2_CENTER);
        message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
    }

    public void parkAtOutsideLane() {
        actionString = "Park";
        message = String.format("start (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
        //
        // assumption is that starting position is approximately ( -43, 18)
        // therefore we are fairly straight angle to the outside lane
        //
        // now go park under the skybridge
        nav.goToPosition(FieldSkystone.X_ORIGIN, FieldSkystone.TILE_1_CENTER);
            message = String.format("end (%.1f,%.1f)", globalPosition.getXinches(), globalPosition.getYinches());
    }

    public void startNearBuildZoneAndGoToFoundation() {
        //
        // Staring position is green wheels towards quarry, robot placed on build zone side of the skybridge
        // BLUE: globalPosition.initGlobalPosition(-14.0, 9.0, 0.0);
        // RED : globalPosition.initGlobalPosition(+14.0, 9.0, 180.0);
        //
        // Driving in reverse to avoid turn around and crashing into alliance partner robot
        robot.setFrontLiftarm();
        telemetry.update(); // print the new orientation of the robot on driver station
        nav.goToPosition(flipX4Red(-2.0*FieldSkystone.TILE_LENGTH), FieldSkystone.TILE_2_CENTER); // Tried DRIVE_SPEED_FAST here, it resulted in overshooting 20% of times
        robot.setFrontIntake();

        // turn robot back towards foundation
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_NEG_Y_AXIS);
        // OBSOLETE: gyroRotate has been replaced by odometryRotate at 2nd tournament. However with timeout it could be resurrected
        // nav.gyroRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_NEG_Y_AXIS), MecaBotMove.ROTATE_SPEED_DEFAULT);

        // move backwards to touch the foundation edge
        nav.odometryMoveForwardBack(-6, MecaBotMove.DRIVE_SPEED_SLOW);
        telemetry.update(); // print the new orientation of the robot on driver station
    }

}


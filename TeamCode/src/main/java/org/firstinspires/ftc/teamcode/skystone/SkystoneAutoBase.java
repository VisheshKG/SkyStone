package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.purepursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

import org.firstinspires.ftc.teamcode.skystone.FieldSkystone.AllianceColor;

/**
 * Each floor tile is 23.5 inch square (counting tabs on one side and not on the other side)
 * Each floor tile with all side tabs cut off is 22.75 inch square
 * The tabs add 0.75 to tile width on each side.
 * Field width = 23.5 * 6 - 0.75 = 70.25 each side square
 *
 * Robot is 18x18 square. Robot (x,y) position is at the center of the robot.
 */

public abstract class SkystoneAutoBase extends LinearOpMode {

    // OpMode members here
    protected MecaBot robot;
    protected MecaBotMove nav;
    protected OdometryGlobalPosition globalPosition;

    protected AllianceColor aColor;

    boolean haveSkystone = false;

    protected double flipX4Red(double value) {
        return (aColor == AllianceColor.BLUE) ? value : -value;
    }

    protected double flipAngle4Red(double value) {
        if (aColor == AllianceColor.RED) {
            value = MathFunctions.angleWrap(value + 180);
        }
        return value;
    }
    /*
     * Abstract method, must be implemented by the sub-classes
     */
    public abstract void setOdometryStartingPosition();

    public ColorSensor chooseColorSensorForSkystone() {
        // random choice, the RED and BLUE subclasses should override this method
        return robot.leftColorSensor;
    }

    /**
     * Initialize all hardware and software data structures
     */
    public void initializeOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new MecaBot();
        robot.init(hardwareMap);
        // lof of problems with gyro rotation crashing the problem, disable it
        // robot.initIMU();
        //

        nav = new MecaBotMove(this, robot);
        globalPosition = nav.getPosition();
        setOdometryStartingPosition();

        telemetry.addData("Status", "Initialized");

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
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return nav.getMovementStatus();
                    }
                });
        telemetry.update();

        // start the thread to calculate robot position continuously
        nav.startOdometry();
    }

    /**
     * do everything in autonomous mode
     * detect a skystone, pick it up, transport and delivery to the foundation,
     * move the foundation, go park itself under the skybridge
     */
    public void runFullAutoProgram() {

        positionToDetectSkystone();
        pickupSkystone();
        deliverSkystone();
        moveFoundation();
        parkAtInsideLane();

    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        while (opModeIsActive()) {

            telemetry.update();
        }
    }

    public void positionToDetectSkystone() {
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

    public void pickupSkystone() {

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

        // Driving in reverse to avoid turn around and crashing into alliance partner robot
        robot.setFrontLiftarm();
        telemetry.update(); // print the new orientation of the robot on driver station
        // destination is the centered on tile in front of center of foundation
        //nav.goToXPosition(flipX4Red(-47), MecaBotMove.DRIVE_SPEED_SLOW);
        nav.goToXPosition(flipX4Red(-55));
        robot.setFrontIntake();

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
            robot.moveLiftArmOutside();
        }

        // move backwards to touch the foundation edge
        nav.odometryMoveForwardBack(-8, MecaBotMove.DRIVE_SPEED_SLOW);
        telemetry.update(); // print the new orientation of the robot on driver station

    }

    public void moveFoundation() {
        // clamp down on the foundation
        robot.grabFoundation();
        // DO NOT REMOVE this sleep() the clamps take a long time, if we don't sleep the robot moves away before clampiong.
        sleep(1000);

        // bring the foundation towards the build zone. When we rotate the foundation in next step,
        // its corner will be in build zone when pushed against the scoreboard wall
        //nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_SLOW);
        //nav.goToPosition(flipX4Red(-38), 18, MecaBotMove.DRIVE_SPEED_DEFAULT, false);
        // trying different approach
        nav.odometryRotateToHeading(flipAngle4Red(-85), MecaBotMove.ROTATE_SPEED_DEFAULT, MecaBotMove.TIMEOUT_SHORT, false);
        nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_FAST);

        // rotate with the foundation to be square with the walls
        nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS), MecaBotMove.ROTATE_SPEED_FAST, MecaBotMove.TIMEOUT_SHORT, false);
/*
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // The gyro initialization CANNOT be controlled by software. It initializes hardware at ZERO angle on program init.
        // GYRO angle is ZERO towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_DEFAULT);
        //nav.encoderTurn(40, true, MecaBotMove.DRIVE_SPEED_SLOW);
*/

        // enough time elapsed in foundation movement, the lift arm must be out now, release skystone
        if (haveSkystone) {
            robot.releaseStoneWithClaw();
            robot.moveLiftArmInside();
        }
        // drive backwards to push the foundation against the scoreboard wall
        // foundation is 18.5 and half robot is 9
        //nav.encoderMoveForwardBack(-4);
        // foundation has been repositioned, release the clamps
        robot.releaseFoundation();
    }

    public void parkAtInsideLane() {
        // go to middle of a tile in inside lane
        //nav.goToPosition(flipX4Red(-23), 35);
        // straighten up to travel along the X-Axis or the player alliance wall
        //nav.odometryRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS));
/*
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // GYRO angle is ZERO towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_DEFAULT);
*/
        // now go park under the skybridge
        nav.goToPosition(0.0, 35.0);
    }

    public void parkAtOutsideLane() {
        //
        // assumption is that starting position is approximately ( -43, 18)
        // therefore we are fairly straight angle to the outside lane
        //
        // now go park under the skybridge
        nav.goToPosition(0.0, 12.0);
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
        nav.goToPosition(flipX4Red(-47), 35); // Tried DRIVE_SPEED_FAST here, it resulted in overshooting 20% of times
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


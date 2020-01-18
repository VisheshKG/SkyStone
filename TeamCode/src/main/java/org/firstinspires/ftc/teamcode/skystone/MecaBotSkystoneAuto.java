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

public abstract class MecaBotSkystoneAuto extends LinearOpMode {

    // OpMode members here
    protected MecaBot robot;
    protected MecaBotMove nav;
    protected OdometryGlobalPosition globalPosition;

    protected AllianceColor aColor;

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

    public abstract ColorSensor chooseColorSensorForSkystone();

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
        robot.initIMU();

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
        //pickupSkystone();
        deliverSkystone();
        moveFoundation();
        parkAtInsideLane();

    }

    // for testing mainly, at the end wait for driver to press STOP, meanwhile
    // continue updating odometry position of the manual movement of the robot
    public void waitForStop() {

        while (opModeIsActive()) {
//            nav.setFrontNormal();
//            sleep(1000);
//            telemetry.update();
//            nav.setFrontReversed();
//            sleep(1000);
            telemetry.update();
        }
    }

    public void positionToDetectSkystone() {
        // stone quarry is 47 inches from the BLUE/RED wall, 48 inches from the audience wall
        // move sideways towards the stone quarry (caution we may hit the skybridge side support)
        // movement is towards +ve Y axis LEFT for BLUE , RIGHT for RED
        nav.odometryMoveRightLeft(flipX4Red(-27), MecaBotMove.DRIVE_SPEED_SLOW);

        // ensure robot direction is straight down the stone quarry
// disabled since we have to goToPosition() anyway after this, no use of aligning to walls.
//        nav.gyroRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_POS_X_AXIS), MecaBotMove.DRIVE_SPEED_MIN);

    }

    public void pickupSkystone() {

        // look for skystone in the stone quarry containing 6 stones
        // choose the correct side color sensor depending on our alliance color
        ColorSensor cs = chooseColorSensorForSkystone();

        // 1st skystone position is special because it cannot be picked up straight front,
        // the skybridge is in the way. We have to grab it at 45% angle.
        nav.odometryMoveForwardBack(5, MecaBotMove.DRIVE_SPEED_SLOW);
        // check for skystone
        if (isSkystone(cs)) {
            telemetry.addData("Detected Skystone at position # ", 1);
            telemetry.update();
        }

        // look for skystones in #2 to #6 position
        for (int i = 2; i <= 6; i++) {
            // move forward one stone length at a time
            nav.odometryMoveForwardBack(8, MecaBotMove.DRIVE_SPEED_SLOW);
            // check for skystone
            if (isSkystone(cs)) {
                // take action to pickup skystone
                telemetry.addData("Detected Skystone at position # ", i);
                telemetry.update();
                break;
            }
        }
        sleep(5000);
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
        robot.setFrontReversed();
        telemetry.update(); // print the new oritentation of the robot on driver station
        // destination is the centered on tile in front of center of foundation
        nav.goToPosition(flipX4Red(-47), 32, MecaBotMove.DRIVE_SPEED_SLOW);
        robot.setFrontNormal();
        sleep(1000);

        // turn towards foundation
        nav.gyroRotateToHeading(flipAngle4Red(FieldSkystone.ANGLE_NEG_Y_AXIS), MecaBotMove.ROTATE_SPEED_SLOW);

        // practical observation note: Rotation decrements the y-position by 3 inches
        // robot position ~ y=32, robot half length = 8.5, foundation position ~ y=47
        // The above for BLUE field, need to update observation for RED field rotation

        // move backwards to touch the foundation edge
        nav.odometryMoveForwardBack(-8, MecaBotMove.DRIVE_SPEED_SLOW);
        telemetry.update(); // print the new oritentation of the robot on driver station
        sleep(2000);

    }

    public void moveFoundation() {
        // clamp down on the foundation
        robot.grabFoundation();
        // DO NOT REMOVE this sleep() the clamps take a long time, if we don't sleep the robot moves away before clampiong.
        sleep(2000);

        // bring the foundation towards the build zone. When we rotate the foundation in next step,
        // its corner will be in build zone when pushed against the scoreboard wall
        nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_SLOW);

        // rotate with the foundation to be square with the walls
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // The gyro initialization CANNOT be controlled by software. It initializes hardware at ZERO angle on program init.
        // GYRO angle is ZERO towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_SLOW);
        //nav.encoderTurn(40, true, MecaBotMove.DRIVE_SPEED_SLOW);

        // drive backwards to push the foundation against the scoreboard wall
        // foundation is 18.5 and half robot is 9
        nav.encoderMoveForwardBack(-20, MecaBotMove.DRIVE_SPEED_SLOW);
        // foundation has been repositioned, release the clamps
        robot.releaseFoundation();
    }

    public void parkAtInsideLane() {
        // go to middle of a tile in inside lane
        nav.goToPosition(flipX4Red(-24), 33, MecaBotMove.DRIVE_SPEED_SLOW);
        // straighten up to travel along the X-Axis or the player alliance wall
        // CAUTION CAUTION -- The GYRO Angle DOES NOT MATCH the ODOMETRY Angle for the RED side.
        // GYRO angle is ZERO towards the stone quarry for BOTH BLUE and RED sides. DO NOT flipAngle4Red() here
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_SLOW);
        // now go park under the skybridge
        nav.goToXPosition(0.0);
    }

}


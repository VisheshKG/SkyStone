package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

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

    public abstract void setOdometryStartingPosition();

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
        //parkAtInsideLane();

        // for testing mainly, at the end continue updating odometry position
        // so we can get readouts of the manual movement of the robot
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
        nav.odometryMoveRightLeft(-27, MecaBotMove.DRIVE_SPEED_SLOW);

        // ensure robot direction is straight down the stone quarry
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.DRIVE_SPEED_SLOW);

    }

    public void pickupSkystone() {
        // look for skystone in the stone quarry containing 6 stones
        for (int i = 1; i <= 6; i++) {
            // move forward one stone length at a time
            nav.odometryMoveForwardBack(8, MecaBotMove.DRIVE_SPEED_SLOW);
            // check for skystone
            if (isSkystone()) {
                // take action to pickup skystone
                telemetry.addData("Detected Skystone at position # ", i);
                telemetry.update();
                break;
            }
        }
        nav.gyroRotateToHeading(-45.0);
        sleep(5000);
    }

    public boolean isSkystone() {
        ColorSensor cs = robot.leftColorSensor;

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
        sleep(2000);
        // destination is the center of tile in front of foundation
        nav.goToPosition(-50, 35, MecaBotMove.DRIVE_SPEED_SLOW);  // drive default speed
        robot.setFrontNormal();
        sleep(5000);

        // turn towards foundation
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_NEG_Y_AXIS, MecaBotMove.ROTATE_SPEED_SLOW);

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
        // the clamps take a long time, if we don't sleep the robot moves away before clampiong.
        sleep(2000);
        // rotate the foundation such that its corner will be in build zone when pushed against the scoreboard wall
        nav.odometryMoveForwardBack(20, MecaBotMove.DRIVE_SPEED_SLOW);
        sleep(2000);

        // rotate to be square with the walls
//        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS);
        nav.encoderTurn(40, true, MecaBotMove.DRIVE_SPEED_SLOW);

        sleep(2000);
        // drive backwards to push the foundation against the scoreboard wall
        // foundation is 18.5 and half robot is 9
        nav.moveForwardBack(-20, MecaBotMove.DRIVE_SPEED_SLOW);
        // foundation has been repositioned, release the clamps
        robot.releaseFoundation();
        sleep(10000);

    }

    public void parkAtInsideLane() {
        // go to middle of a tile in inside lane
        nav.goToPosition(-35, 35);
        // straighten up to travel along the X-Axis or the player alliance wall
        nav.gyroRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS, MecaBotMove.ROTATE_SPEED_SLOW);
        // now go park under the skybridge
        nav.goToXPosition(0.0);
    }

}


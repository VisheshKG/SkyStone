package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.skystone.FieldSkystone;
import static org.firstinspires.ftc.teamcode.purepursuit.MathFunctions.angleWrapRad;


public class MecaBotMove {

    enum WheelPosition { LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK }

    static final int    MOTOR_TICK_COUNT    = 560; // we are using REV HD Hex Planetary 20:1 for drive train
    static final float  mmPerInch           = 25.4f;
    static final double WHEEL_DIA           = 75 / mmPerInch;  // REV mecanum wheels have 75 millimeter diameter
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIA;
    static final double ENCODER_TICKS_PER_INCH      = MOTOR_TICK_COUNT / WHEEL_CIRCUMFERENCE;
    static final int    ENCODER_TICKS_ERR_MARGIN    = 50;
    static final double DEFAULT_SPEED       = 0.6;  //default wheel speed, same as motor power
    static final double OUTER_TO_INNER_TURN_SPEED_RATIO = 6.0;

    static final double X_PARK_INNER_OUTER  = 9.0;

    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private MecaBot             robot;          // Access to the Robot hardware
    private OdometryGlobalPosition globalPosition; // Robot global position tracker
    private Thread              globalPositionThread;

//    private static double       curX=0;
//    private static double       curY=0;

    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
        globalPosition = robot.initOdometry();
    }

    // Access methods
    public OdometryGlobalPosition getPosition() {
        return globalPosition;
    }

    public void startOdometry() {

        globalPositionThread = new Thread(globalPosition);
        globalPositionThread.start();
    }

    public void stopOdometry() {

        globalPosition.stop();
    }

    /**
     * Drive the robot at specified speed towards the specified target position on the field.
     * Note that we are not waiting to reach the target position. This method only sets wheel power
     * in the direction of the target position and must be called again repeatedly at an update
     * interval, typically 50ms - 75ms.
     * The current position of the robot obtained using odometry readings and wheel power are both
     * calculated again at each call to this method.
     *
     * @param x     Target position global x coordinate value (inches)
     * @param y     Target position global y coordinate value (inches)
     * @param speed Speed/Power used to drive. Must be in range of -1.0 <= speed <= 1.0
     * @return <em>true</em> if sucessfully issued command to robot to drive, <em>false</em> if reached the destination
     */
    public boolean goTowardsPosition(double x, double y, double speed) {

        double distanceToPosition = Math.hypot(globalPosition.getXinches() - x,  globalPosition.getYinches() - y);
        double absoluteAngleToPosition = Math.atan2(y - globalPosition.getYinches(), x - globalPosition.getXinches());
        double relativeAngleToPosition = angleWrapRad(absoluteAngleToPosition - globalPosition.getOrientationRadians());

        // when within 1 feet (12 inches) of target, reduce the speed proportional to remaining distance to target
        double drivePower = Range.clip(distanceToPosition / 12, 0.2, 1.0) * speed;
        // however absolute minimium 0.15 power is required otherwise the robot cannot move the last couple of inches
        if (drivePower < 0.15)
            drivePower = 0.15;

        // set turnspeed proportional to the amount of turn required, however beyond 30 degrees turn, full speed is ok
        // note here that positive angle means turn left (since angle is measured counter clockwise from X-axis)
        // this must match the behavior of MecaBot.DriveTank() method used below.
        double turnPower = Range.clip(relativeAngleToPosition / Math.toRadians(30), -1.0, 1.0) * speed;
        // no minimum for turnPower until robot auto driving tests indicate a need.

        myOpMode.telemetry.addData("Driving to location ", "(X %3.2f, Y %3.2f)", x, y);
        myOpMode.telemetry.addLine("Target ").addData("Distance", " %4.2f in", distanceToPosition).addData("Angle", " %4.2f deg", Math.toDegrees(relativeAngleToPosition));
        myOpMode.telemetry.addLine("Power: ").addData("drive", " %.2f", drivePower).addData("turn", "%.2f", turnPower);

        // let's stop driving when within a short distance of the destination. This threshold may need to be tuned.
        // A threshold is necessary to avoid oscillations caused by overshooting of target position.
        // This check could be done early in the method, however it is done towards end deliberately to get telemetry readouts
        if (distanceToPosition < 1) {
            robot.stopDriving();
            return false;
        }

        robot.driveTank(drivePower, turnPower);

        return true;
    }

    /*
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     */
    public void moveForwardBack(double inches) {
        moveDistance( inches, false, DEFAULT_SPEED);
    }

    public void moveForwardBack(double inches, double speed) {
        moveDistance( inches, false, speed);
    }

   /*
    * Move robot left or right, +ve distance moves right, -ve distance moves left
    */
    public void moveRightLeft(double inches) {
        moveDistance(inches, true, DEFAULT_SPEED);
    }

    public void moveRightLeft(double inches, double speed) {
        moveDistance(inches, true, speed);
    }

    /*
     * Move robot left or right, +ve distance moves LEFT, -ve distance moves RIGHT
     */
    public void moveLeftRight(double inches) {
        // maybe needed to compensate for weak movements
        if (inches < 0){    //right over drive by a multiple
            inches=inches * FieldSkystone.rightMultiple;
        }
        moveDistance(inches * -1.0, true, DEFAULT_SPEED);
    }

    public void moveLeftRight(double inches, double speed) {
        moveDistance(inches * -1.0, true, speed);
    }

    /*
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     */
    private void moveDistance(double inches, boolean mecanumSideways, double speed) {

        //convert inches to tick counts
        int driverEncoderTarget = (int) (ENCODER_TICKS_PER_INCH * inches);

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForwardOrRight is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
            rightFront = -driverEncoderTarget;  // drive backward for inside
            rightBack = driverEncoderTarget;    // drive forward for inside
            leftFront = driverEncoderTarget;    // drive forward for outside
            leftBack = -driverEncoderTarget;    // drive backward for outside
        }
        // same code above works for mecanum move Left also. False value of goForwardOrRight already flipped the sign above

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of the motors to whatever speed is needed
        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(WheelPosition.RIGHT_FRONT, leftFront, leftBack, rightFront, rightBack);

    }

    // Rotate around Robot own center
    public void encoderRotate(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot,
        // counterClockwise means Right wheels turning forward, Left wheels turning backwards

        int ticks = (int) (ENCODER_TICKS_PER_INCH * inches);

        int leftFront = counterClockwise ? -ticks : +ticks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? +ticks : -ticks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderRotate(double inches, boolean counterClockwise) {
        encoderRotate(inches, counterClockwise, DEFAULT_SPEED);
    }

    // Turn in an arc
    // Assume robot is touching the outer edge of a tile and we want to rotate it in a circular arc
    // This is not rotating around its center axis but an arc with center of circle outside the robot where the tiles meet
    // Outer wheels run along outer circle and inner wheels run along an inner circle

    public void encoderTurn(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot, counterClockwise means Right wheels on outer circle
        double outerWheelInches = inches;
        double innerWheelInches = inches / OUTER_TO_INNER_TURN_SPEED_RATIO;

        int outerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * outerWheelInches);
        int innerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * innerWheelInches);

        int leftFront = counterClockwise ? innerWheelTicks : outerWheelTicks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? outerWheelTicks : innerWheelTicks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double wheelPower = Range.clip(speed, 0.0, 1.0);
        double insideWheelPower = wheelPower / OUTER_TO_INNER_TURN_SPEED_RATIO;
        if (counterClockwise) {
            robot.setDrivePower(insideWheelPower, wheelPower);
        } else {
            robot.setDrivePower(wheelPower, insideWheelPower);
        }

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", outerWheelInches).addData("inner", innerWheelInches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderTurn(double inches, boolean counterClockwise) {
        encoderTurn(inches, counterClockwise, DEFAULT_SPEED);
    }

    public void waitToReachTargetPosition(WheelPosition dominantWheel, int leftFront, int leftBack, int rightFront, int rightBack) {

        // Loop until motors are no longer busy.
        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {

            myOpMode.telemetry.addLine("Target position | ").addData("LF", leftFront).addData("RF", rightFront);
            myOpMode.telemetry.addLine("Target position | ").addData("LB", leftBack).addData("RB", rightBack);
            myOpMode.telemetry.addLine("Current position | ").addData("LF", robot.leftFrontDrive.getCurrentPosition()).addData("RF", robot.rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addLine("Current position | ").addData("LB", robot.leftBackDrive.getCurrentPosition()).addData("RB", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();

            //encoder reading a bit off target can keep us in this loop forever, so given an error margin here
            if ((dominantWheel == WheelPosition.LEFT_FRONT) && Math.abs(robot.leftFrontDrive.getCurrentPosition() - leftFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.LEFT_BACK) && Math.abs(robot.leftBackDrive.getCurrentPosition() - leftBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_FRONT) && Math.abs(robot.rightFrontDrive.getCurrentPosition() - rightFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_BACK) && Math.abs(robot.rightBackDrive.getCurrentPosition() - rightBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();
        myOpMode.telemetry.addData("DONE driving LF position=",robot.leftFrontDrive.getCurrentPosition());
        // Turn off RUN_TO_POSITION
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.sleep(250);

    }

    public boolean isUnderBridge(){
        ColorSensor cs = robot.rightColorSensor;

        myOpMode.telemetry.addData("Blue Reading=", cs.blue());
        myOpMode.telemetry.addData("Red Reading=", cs.red());
        //myOpMode.telemetry.addData("Alpha Reading=", cs.alpha());
        myOpMode.telemetry.update();

        if (cs.blue() > 1000) {
            myOpMode.telemetry.addData("Ground is BLUE--Bridge reached", cs.blue());
            return true;
        }
        if (cs.red() > 1000){
            myOpMode.telemetry.addData("Ground is RED--Bridge reached", cs.red());
            return true;
        }

        return false;
    }

/*
 * The code below is obsolete after the 24-NOV-2019 Qualifying Tournament.
 * 1. The Robot does not have a side arm, the robot heading will always towards the stone side for both RED and BLUE
 * 2. Since we are using odometry we cannot flip the X-axis positive direction between RED and BLUE.
 */


/**
 *  Method to drive the robot from one point marked by coordinate curX, curY to another targetX, target Y.
 *  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
 *  with Y pointing to the middle of the field and X pointing to the stone side.
 *  Robot move in Y direction first followed by X.
 *    @param targetX  target X coordinate value
 *    @param targetY  target Y coordinate value
 *    @param curX     start X coordinate value
 *    @param curY     start Y coordinate value
 *    @param headXpositive   true when robot forward heading is along positive X axis
 *                           set this to true for RED alliance, false for BLUE alliance
 */

    public void moveYX(double targetX, double targetY, double curX, double curY, boolean headXpositive){
        double distanceMarginInch=0.1;    //minimum required distance to invoke the move
        double xdist=targetX-curX;
        double ydist=targetY-curY;
        if (!headXpositive){
            //blue side: negative x require robot to go forward
            xdist=-xdist;
        }
        ydist=-ydist;  //both side: positive y movement require robot to go right

        myOpMode.telemetry.addData(" Xdist Ydist", "%.1f %.1f", xdist,ydist);
        if (Math.abs(ydist) > distanceMarginInch){
            moveLeftRight(ydist);
        }
        if (Math.abs(xdist) > distanceMarginInch) {
            moveForwardBack(xdist);
        }
    }

    /**
     *  Method to park the robot under the bridge. It used the method moveYX() method
     *  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
     *  with Y pointing to the middle of the field and X pointing to the stone side.
     *    @param curX     start X coordinate value
     *    @param curY     start Y coordinate value
     *    @param parkInside      park under the bridge on inside tile towards center of field (or outside tile towards perimeter)
     *    @param headXpositive   true when robot forward heading is along positive X axis
     *                           set this to true for RED alliance, false for BLUE alliance
     */
    public void goPark(double curX, double curY, boolean parkInside, boolean headXpositive){

        //adjusted temporarily for Red Alliance Parking
        double toY = parkInside ? FieldSkystone.bridgeY- FieldSkystone.parkingMarginR:
                 FieldSkystone.robotWidth+ FieldSkystone.parkingMarginL;
        double toX = headXpositive? -5:X_PARK_INNER_OUTER;

        myOpMode.telemetry.addData("Parking target X Y", "%.1f %.1f", toX,toY);
        moveYX(toX,toY,curX,curY,headXpositive);
    }

}
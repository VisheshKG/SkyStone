package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class MecaBotMove {

    private final static int    MOTOR_TICK_COUNT = 560; // we are using REV HD Hex Planetary 20:1 for drive train
    private static final float  mmPerInch        = 25.4f;
    private final static double WHEEL_DIA = 75 / mmPerInch;  //in millimeter
    private LinearOpMode  myOpMode;       // Access to the OpMode object
    private MecaBot       robot;        // Access to the Robot hardware
    private double speed=0.5;
    private final double LOWSPEED = 0.2;
    private final double HIGHSPEED = 0.6;
    //current location: origin is at red/blue wall center with x pointing to stone side and y to center of field
    private static double curX=0;
    private static double curY=0;

    private static double wheelPower=0.6;  //default wheel speed


    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
    }

    public double getWheelMoveInches(){
        int tickMoved=robot.rightBackDrive.getCurrentPosition();
        double inchMoved= tickMoved/MOTOR_TICK_COUNT / WHEEL_DIA;
        return inchMoved;
    }

    public void setSpeedWheel(double speed) {
        wheelPower=speed;
    }

    public double getCurX(){ return curX;}
    public double getCurY(){ return curY;}
    public void setCurX(double x){curX=x;}
    public void setCurY(double y){curY=y;}
    /*
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     */
    public void moveForwardBack(double inches) {
        moveDistance( inches, false);
    }

   /*
    * Move robot left or right, +ve distance moves right, -ve distance moves left
    */
    public void moveLeftRight(double inches) {
        moveDistance(inches, true);
    }

    /*
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     */
    private void moveDistance(double inches, boolean mecanumSideways) {

        robot.resetDriveEncoder();

        //cw: convert millimeter to tick counts
        double circumference = Math.PI * WHEEL_DIA;
        double numRotation = inches/circumference;
        int driverEncoderTarget = (int) (MOTOR_TICK_COUNT * numRotation);

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForwardOrRight is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
/*
            leftFront = driverEncoderTarget;    // drive forward for outside
            leftBack = -driverEncoderTarget;    // drive backward for outside
            rightFront = -driverEncoderTarget;  // drive backward for inside
            rightBack = driverEncoderTarget;    // drive forward for inside
            */

            //When driverEncoderTarget is positive, left side drive into each other, robot move left
            leftFront = -driverEncoderTarget;
            leftBack = driverEncoderTarget;
            rightFront = driverEncoderTarget;
            rightBack = -driverEncoderTarget;
        }
        // same code above works for mecanum move Left also. False value of goForwardOrRight already flipped the sign above

        // set target position for encoder Drive
        robot.leftFrontDrive.setTargetPosition(leftFront);
        robot.leftBackDrive.setTargetPosition(leftBack);
        robot.rightFrontDrive.setTargetPosition(rightFront);
        robot.rightBackDrive.setTargetPosition(rightBack);

        // Set the motors to run to the necessary target position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of the motors to whatever speed is needed
        robot.driveStraight(wheelPower);

        // Loop until both motors are no longer busy.
        myOpMode.telemetry.addData("Encoder Drive", "Driving for %.2f inches",inches);
        myOpMode.telemetry.addData("=======Encoder target ticks=", rightBack);
        //myOpMode.telemetry.update();

        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            // no need to do any checks
            // the documentation says that motors stop automatically in RUN_TO_POSITION mode and isBusy() will return false after that
            //myOpMode.telemetry.addData("Encoder Drive", "Driving %.2f inches = %d encoder ticks", inches, driverEncoderTarget);
            myOpMode.telemetry.addData("rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
            //encoder reading a bit off target can keep us in this loop forever, so given an error margin here
            int errMargin=50;
            if (Math.abs(robot.leftFrontDrive.getCurrentPosition() - leftFront) < errMargin &&
                    Math.abs(robot.leftBackDrive.getCurrentPosition() - leftBack) < errMargin &&
                    Math.abs(robot.rightFrontDrive.getCurrentPosition() - rightFront) < errMargin &&
                    Math.abs(robot.rightBackDrive.getCurrentPosition() - rightBack) < errMargin ){
                break;
            }
            //myOpMode.sleep(10);
        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();

        myOpMode.telemetry.addData("--------Stopped at Target, rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    // Rotate
    public void turn(int degrees, boolean counterClockwise) {
    }

    // Claw and Damper movements
    public void grabTheStone(){
        robot.grabStoneWithSidearm();
    }

    public void releaseTheStone(){
        robot.releaseStoneWithSidearm();
    }

    public void grabFoundation(){
        robot.grabFoundation();
    }

    public void releaseFoundation(){
        robot.releaseFoundation();
    }

    public boolean isUnderBridge(){
        ColorSensor cs = robot.groundColorSensor;

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

        if (Math.abs(ydist) > distanceMarginInch){
            moveLeftRight(ydist);
        }
        if (Math.abs(xdist) > distanceMarginInch) {
            moveForwardBack(xdist);
        }
    }

}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
    }

    public double getWheelMoveInches(){
        int tickMoved=robot.rightBackDrive.getCurrentPosition();
        double inchMoved= tickMoved/MOTOR_TICK_COUNT * WHEEL_DIA;
        return inchMoved;
    }

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

        // Vishesh todo Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target position

        // flip direction for reverse (this also applies for Left sideways when mecanumSideways is true)
        driverEncoderTarget = -driverEncoderTarget;  // reverse the encoder target direction

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForwardOrRight is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
            leftFront = driverEncoderTarget;    // drive forward for outside
            leftBack = -driverEncoderTarget;    // drive backward for outside
            rightFront = -driverEncoderTarget;  // drive backward for inside
            rightBack = driverEncoderTarget;    // drive forward for inside
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
        robot.driveStraight(HIGHSPEED);

        // Loop until both motors are no longer busy.
        myOpMode.telemetry.addData("Encoder Drive", "Driving for %.2f inches", inches);
        myOpMode.telemetry.update();

//        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
        while (robot.rightBackDrive.isBusy()) {
            // no need to do any checks
            // the documentation says that motors stop automaticaqlly in RUN_TO_POSITION mode and isBusy() will return false after that
            myOpMode.telemetry.addData("Encoder Drive", "Driving %.2f inches = %d encoder ticks", inches, driverEncoderTarget);
            myOpMode.telemetry.addData("rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(50);
        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();

        myOpMode.telemetry.addData("Stopped at Target, rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
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
}
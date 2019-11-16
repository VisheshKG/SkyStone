package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class MecaBotMove {

    private final static double diaWheel = 75;  //in millimeter
    private final static int MOTOR_TICK_COUNT =1120;
    private static final float mmPerInch        = 25.4f;
    private LinearOpMode  myOpMode;       // Access to the OpMode object
    private MecaBot       robot;        // Access to the Robot hardware
    private double speed=0.5;
    private final double LOWSPEED = 0.2;
    private final double HIGHSPEED = 0.8;

    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
    }

    //reset the encoders back to 0
    public void resetWheelCounter() {
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSpeed(String level){
        if (level.equals("LOW")){
            speed = LOWSPEED;
        }else{
            speed = HIGHSPEED;
        }
    }

    public void setSpeed(double driveSpeed){
        speed=driveSpeed;
    }

    public void stopWheels(){
        robot.leftBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }

    public double getWheelMoveInches(){
        int tickMoved=robot.rightBackDrive.getCurrentPosition();
        double mmMoved= tickMoved/MOTOR_TICK_COUNT * diaWheel;
        double inchMoved=mmMoved/mmPerInch;
        return inchMoved;
    }

    public void moveForward(int inches) {
        double mm = inches * mmPerInch;
        moveForwardBack(mm, true);
    }

    public void moveBackward(double inches) {
        double mm = inches * mmPerInch;
        moveForwardBack(mm, false);
    }

    public void moveLeft(double inches) {
        double mm = inches * mmPerInch;
        moveLeftRight(mm, true);
    }

    public void moveRight(double inches) {
        double mm = inches * mmPerInch;
        moveLeftRight(mm, false);
    }

    private void moveForwardBack(double mm, boolean driveForward) {

        resetWheelCounter();

        //cw: convert millimeter to tick counts
        double circumference = 3.14*diaWheel;
        double numRotation = mm/circumference;
        int driverEncoderTarget = (int) (MOTOR_TICK_COUNT * numRotation);

        // Vishesh todo Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target position

        robot.leftBackDrive.setTargetPosition(driverEncoderTarget);
        robot.leftFrontDrive.setTargetPosition(driverEncoderTarget);
        robot.rightBackDrive.setTargetPosition(driverEncoderTarget);
        robot.rightFrontDrive.setTargetPosition(driverEncoderTarget);

        // Set the power of the motors to whatever speed is needed
        if (! driveForward){
            speed = -speed;  //Q:reverse direction?  --cwm
            myOpMode.telemetry.addData("Drive Backward", "None");
        }
        robot.leftBackDrive.setPower(speed);
        robot.leftFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);

        // Set the motors to run to the necessary target position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Loop until both motors are no longer busy.
        myOpMode.telemetry.addData("Going forward/back by mm=", mm);
        myOpMode.telemetry.update();
        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            /*
            if (robot.leftBackDrive.getCurrentPosition() > robot.leftBackDrive.getTargetPosition() - 10 && robot.leftBackDrive.getCurrentPosition() < robot.leftBackDrive.getTargetPosition() + 10) {
                if (robot.rightBackDrive.getCurrentPosition() > robot.rightBackDrive.getTargetPosition() - 10 && robot.rightBackDrive.getCurrentPosition() < robot.rightBackDrive.getCurrentPosition() + 10) {
                    break;
                }
            }
             */
            myOpMode.telemetry.addData("rightBackDrive cur position= ", robot.rightBackDrive.getCurrentPosition());
        }

        // Stop powering the motors - robot has moved to intended position
        robot.leftBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        myOpMode.telemetry.update();
    }

    // Move robot left or right
    public void moveLeftRight(double mm, boolean left) {

    }

    // Rotate
    public void turn(int degrees, boolean counterClockwise) {
    }

    // Claw and Damper movements
    public void grabTheStone(){
    }

    public void releaseTheStone(){
    }

    public void grabFoundation(){}

    public void releaseFoundation(){}


}
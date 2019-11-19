package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class MecaBotMove {

    private final static double WHEEL_DIA = 75;  //in millimeter
    private final static int    MOTOR_TICK_COUNT = 560;  // 1120/2
    private static final float  mmPerInch        = 25.4f;
    private LinearOpMode  myOpMode;       // Access to the OpMode object
    private MecaBot       robot;        // Access to the Robot hardware
    private double speed=0.5;
    private final double LOWSPEED = 0.2   ;
    private final double HIGHSPEED = 0.8;

    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
    }

    public double getWheelMoveInches(){
        int tickMoved=robot.rightBackDrive.getCurrentPosition();
        double mmMoved= tickMoved/MOTOR_TICK_COUNT * WHEEL_DIA;
        double inchMoved=mmMoved/mmPerInch;
        return inchMoved;
    }

    public void moveForward(double inches) {
        double mm = inches * mmPerInch;
        moveForwardBack(mm, true);
    }

    public void moveBackward(double inches) {
        double mm = inches * mmPerInch;
        myOpMode.telemetry.addData(">>moveBackward:",mm);
        myOpMode.telemetry.update();
        moveForwardBack(mm, false);
        myOpMode.telemetry.addData("<<moveBackward:",mm);
        myOpMode.telemetry.update();
    }

    private void moveForwardBack(double mm, boolean goForward) {
        moveDistance( mm, goForward, false);
    }


    public void moveRight(double inches) {
        double mm = inches * mmPerInch;
        moveLeftRight(mm, true);
    }

    public void moveLeft(double inches) {
        double mm = inches * mmPerInch;
        moveLeftRight(mm, false);
    }

    // Move robot left or right
    public void moveLeftRight(double mm, boolean goRight) {
        boolean reverseIt=!goRight;   //todo: workaround bug
        moveDistance(mm, reverseIt, true);
    }

    //DEBUG: test any kinds of movements here. This is called by Auto1 opmode
    public void  testMove(){
     //testAllwheelsNoEncoder();
     //testOneMotorEncoder(1);
        //grabTheStone();

        double testDistance=2* Math.PI * WHEEL_DIA/mmPerInch;
        //testDistance=11.75;
        testDistance=12.50;
        moveRight(testDistance);
        //moveLeft(testDistance);
        //moveBackward(testDistance);
    }

    //DEBUG: move all wheels with equal power
    private void  testAllwheelsNoEncoder(){
        myOpMode.telemetry.addData(">>test all wheels:","No Encoder");
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setWheelSpeed(LOWSPEED);
        myOpMode.sleep(4000);
        robot.stopDriving();
        myOpMode.telemetry.addData("<<testMove:","none");
        myOpMode.telemetry.update();
    }

    private void setWheelEncoder(DcMotor aWheelDrive, int targetTickCT){
        aWheelDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aWheelDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aWheelDrive.setTargetPosition(targetTickCT);
    }
    //test one motor with encoder
private void testOneMotorEncoder(int numRotation){
    myOpMode.telemetry.addData(">>test one wheel:","Encoder");
    robot.resetDriveEncoder();
    int driverEncoderTarget = MOTOR_TICK_COUNT * numRotation;
    DcMotor aWheelDrive=robot.rightBackDrive;
    setWheelEncoder(aWheelDrive, driverEncoderTarget);
    aWheelDrive=robot.leftBackDrive;
    setWheelEncoder(aWheelDrive, driverEncoderTarget);
    myOpMode.telemetry.addData("Start drive rightBack target rotation =", 1);
    myOpMode.telemetry.update();
    myOpMode.sleep(2000);

    // Set the power of the motors to whatever speed is needed
    //robot.driveStraight(LOWSPEED);
    setWheelSpeed(LOWSPEED);

    while (robot.rightBackDrive.isBusy()) {
        myOpMode.telemetry.addData("rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }
    robot.stopDriving();
    double rotActual=robot.rightBackDrive.getCurrentPosition() / MOTOR_TICK_COUNT;
    myOpMode.telemetry.addData("rightBackDrive actual rotation= ", rotActual);
    myOpMode.telemetry.update();
    myOpMode.sleep(10000);
}


    private void setWheelSpeed(double speed) {

        robot.leftFrontDrive.setPower(speed);
        robot.leftBackDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
    }

    private void setDriveMode() {
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // Rotate
    public void turn(int degrees, boolean counterClockwise) {
    }

    // Claw and Damper movements
    public void grabTheStone(){
        robot.grabStoneWithSidearm();
        //cwm
        myOpMode.sleep(5000);
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

    private void moveDistance(double mm, boolean goForwardOrRight, boolean mecanumSideways) {

        robot.resetDriveEncoder();

        //cw: convert millimeter to tick counts
        double circumference = Math.PI * WHEEL_DIA;
        double numRotation = mm/circumference;
        int driverEncoderTarget = (int) (MOTOR_TICK_COUNT * numRotation);
        myOpMode.telemetry.addData("Encoder Drive", "Target = %d", driverEncoderTarget);
        myOpMode.telemetry.addData("numRotation=",numRotation);
        myOpMode.telemetry.update();
        myOpMode.sleep(5000);

        // Vishesh todo Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target position

        // flip direction for reverse (this also applies for Left sideways when mecanumSideways is true)
        if (!goForwardOrRight) {
            driverEncoderTarget = -driverEncoderTarget;  // reverse the encoder target direction
            myOpMode.telemetry.addData("Encoder Drive", "Target = %d", driverEncoderTarget);
        }

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForward is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
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
        robot.driveStraight(LOWSPEED);

        // Loop until both motors are no longer busy.
        myOpMode.telemetry.addData("Driving distance mm = ", mm);
        myOpMode.telemetry.update();

//        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
        while (robot.rightBackDrive.isBusy()) {
            // no need to do any checks
            // the documentation says that motors stop automaticaqlly in RUN_TO_POSITION mode and isBusy() will return false after that
            myOpMode.telemetry.addData("Encoder Drive", "Target = %d", driverEncoderTarget);
            //myOpMode.telemetry.addData("rightBackDrive position = ", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();

        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();

        myOpMode.telemetry.addData("Stopped at Target", "None");
        myOpMode.telemetry.update();
        myOpMode.sleep(5000);
    }


}
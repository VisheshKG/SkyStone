package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 10/4/2019.
 * Modified by Vishesh Goyal on 12/24/2019
 * - adaptation to Team13345 MecaBot hardware
 */
@TeleOp(name = "My Odometry OpMode", group = "Test")
@Disabled
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 242.552133272048492;  // FTC Team 13345 MecaBot encoder has 1440 ticks per rotation, wheel has 48mm diameter
                                                         // (1440 * MM_PER_INCH) / ( Math.PI * 48)

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rightFrontDrive", rbName = "rightBackDrive", lfName = "leftFrontDrive", lbName = "leftBackDrive";
    String verticalLeftEncoderName = "leftIntake", verticalRightEncoderName = "rightIntake", horizontalEncoderName = "horizontalEncoder";

    OdometryGlobalPosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalPosition thread to constantly update the global position coordinates.
        globalPositionUpdate = new OdometryGlobalPosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);

        // Set direction of odometry encoders. PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT HARDWARE
        // Left encoder value, robot forward movement should produce positive encoder count
        // globalPositionUpdate.reverseLeftEncoder();
        // Right encoder value, robot forward movement should produce positive encoder count
        globalPositionUpdate.reverseRightEncoder();
        // Horizontal encoder value, robot right sideways movement should produce positive encoder count values
        globalPositionUpdate.reverseNormalEncoder();

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.getXinches());
            telemetry.addData("Y Position", globalPositionUpdate.getYinches());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.getOrientationDegrees());

            telemetry.addData("Vertical left encoder", globalPositionUpdate.getVerticalLeftCount());
            telemetry.addData("Vertical right encoder", globalPositionUpdate.getVerticalRightCount());
            telemetry.addData("horizontal encoder", globalPositionUpdate.getHorizontalCount());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "MecaBotTeleOp")
public class MecaBotTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    public static final double MIN_SERVO       =  0.0 ;
    public static final double MID_SERVO       =  0.5 ;
    public static final double MAX_SERVO       =  1.0 ;

    /* Declare OpMode members. */
    MecaBot robot = new MecaBot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        telemetry.addData(">", "Hardware initialized");

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive();
            lift();
            intake();
            bumper();
            telemetry.update();
        }
    }

    public void drive() {
        // motor power
        double leftFront = 0;
        double leftBack = 0;
        double rightFront = 0;
        double rightBack = 0;
        double max;

            //if we want to move sideways (MECANUM)
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {

                // if we want to move right sideways, joystick value is positive
                //right inside
                rightFront -= gamepad1.left_stick_x;
                rightBack += gamepad1.left_stick_x;

                //left outside
                leftFront += gamepad1.left_stick_x;
                leftBack -= gamepad1.left_stick_x;

                // if we want to move left, its same code as above, joystick value is negative
            }

            // normal tank movement
            else {

                // basic forward/backwards, run motors at speed controlled by joystick
                leftFront -= gamepad1.left_stick_y;
                leftBack -= gamepad1.left_stick_y;
                rightFront -= gamepad1.left_stick_y;
                rightBack -= gamepad1.left_stick_y;

                // right press on joystick is positive, left press is negative
                // to turn right, add turning joystick to left motors, subtract from right
                // to turn left, same code applies, sign is reversed automatically by joystick value
                leftFront += gamepad1.right_stick_x;
                leftBack += gamepad1.right_stick_x;
                rightFront -= gamepad1.right_stick_x;
                rightBack -= gamepad1.right_stick_x;

           }

            // find the highest power motor and divide all motors by that to preserve the ratio
            // while also keeping the maximum power at 1
            max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
            if (max > 1) {
                leftFront /= max;
                leftBack /= max;
                rightFront /= max;
                rightBack /= max;
            }

            //set motor's power to the values calculated
            robot.leftFrontDrive.setPower(leftFront);
            robot.leftBackDrive.setPower(leftBack);
            robot.rightFrontDrive.setPower(rightFront);
            robot.rightBackDrive.setPower(rightBack);
        }


    public void lift() {
       telemetry.addData(">", "entered lift loop");

       if (gamepad2.left_stick_y > 0 && robot.liftMotor.getCurrentPosition() < robot.LIFT_MAX) {
           robot.liftMotor.setPower(gamepad2.left_stick_y);
       }
       else if (gamepad2.left_stick_y < 0 && robot.liftMotor.getCurrentPosition() > robot.LIFT_MIN) {
           robot.liftMotor.setPower(gamepad2.left_stick_y);
       }
       else {
           robot.liftMotor.setPower(0 );
       }
       telemetry.addData(">", "Lift tick count = " + robot.liftMotor.getCurrentPosition());
       //robot.liftServo.setPosition(robot.liftServo.getPosition() + (gamepad2.right_stick_y / 20));

        // If operator joystick is pushed upwards, move the lift arm outside the robot
       if (gamepad2.right_stick_y > 0) {
           robot.liftServo.setPosition(robot.ARM_OUTSIDE);
           telemetry.addData(">", "right joystick pushed up %5.2f", gamepad2.right_stick_y);

       }
       // If operator joystick is pushed downwards, move the lift arm inside the robot
       else if (gamepad2.right_stick_y < 0) {
           robot.liftServo.setPosition(robot.ARM_INSIDE);
           telemetry.addData(">", "right joystick pushed down %5.2f", gamepad2.right_stick_y);

       }

       //robot.clawRotate.setPosition(robot.clawRotate.getPosition() + (gamepad2.right_trigger / 20) - (gamepad2.left_trigger / 20));

        // If operator right trigger is pressed, rotate claw to Stone pickup position inside robot
       if (gamepad2.right_trigger > 0) {
           robot.clawRotate.setPosition(robot.CLAW_PARALLEL);
           telemetry.addData(">", "right trigger pushed %5.2f", gamepad2.right_trigger);

       }
       // If operator left trigger is pressed, rotate claw perpendicular to Stone pickup position
       else if (gamepad2.left_trigger > 0) {
           robot.clawRotate.setPosition(robot.CLAW_PERPENDICULAR);
           telemetry.addData(">", "left trigger pushed %5.2f", gamepad2.left_trigger);
       }

        if (gamepad2.right_bumper) {
            robot.clawGrab.setPosition(robot.CLAW_CLOSE); // right is grab the stone, claw closed
            telemetry.addData(">", "right bumper pushed");
        }
        else if (gamepad2.left_bumper) {
            robot.clawGrab.setPosition(robot.CLAW_OPEN); // left is release the stone, claw open
            telemetry.addData(">", "left bumper pushed");
        }
    }

    public void intake() {

        if (gamepad1.right_trigger > 0) {
            robot.leftIntake.setPower(-gamepad1.right_trigger);
            robot.rightIntake.setPower(-gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0) {
            robot.leftIntake.setPower(gamepad1.left_trigger);
            robot.rightIntake.setPower(gamepad1.left_trigger);
        }
        else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

    }

    public void bumper() {
        if (gamepad2.a) {
            robot.bumperServo.setPosition(1.0); //open
        }
        else if (gamepad2.b) {
            robot.bumperServo.setPosition(0.5); //close
        }
    }

}

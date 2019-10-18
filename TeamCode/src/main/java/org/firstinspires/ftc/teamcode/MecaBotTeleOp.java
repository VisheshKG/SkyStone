package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "MainTeleOP")
public class MecaBotTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    MecaBot robot = new MecaBot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Say", "Iteration Started");
            telemetry.update();
            drive();
            lift();
            telemetry.addData("Say", "Iteration Complete");
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
 /*           if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {

                // if we want to move right
                if (gamepad1.left_stick_x > 0) {
                    //right inside
                    rightFront -= gamepad1.left_stick_x;
                    rightBack += gamepad1.left_stick_x;

                    //left outside
                    leftFront += gamepad1.left_stick_x;
                    leftBack -= gamepad1.left_stick_x;
                }

                // if we want to move left
                else if (gamepad1.left_stick_x < 0) {
                    //left inside
                    leftFront -= gamepad1.left_stick_x;
                    leftBack += gamepad1.left_stick_x;

                    //right outside
                    rightFront += gamepad1.left_stick_x;
                    rightBack -= gamepad1.left_stick_x;
                }
            }
            // normal tank movement
            else { */

                // basic forward/backwards, joystick forward is negative so negate it
                leftFront += -gamepad1.left_stick_y;
                leftBack += -gamepad1.left_stick_y;
                rightFront += -gamepad1.left_stick_y;
                rightBack += -gamepad1.left_stick_y;

                // add turning joystick to right motors, subtract from left
                leftFront += gamepad1.right_stick_x;
                leftBack += gamepad1.right_stick_x;
                rightFront -= gamepad1.right_stick_x;
                rightBack -= gamepad1.right_stick_x;


 //           }

            // find the highest power motor and divide all motors by that to preserve the ratio
            // while also keeping the maximum power at 1
            max = Math.max(Math.max(leftFront, leftBack), Math.max(rightFront, rightBack));
            if (max != 0) {
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
        if (gamepad2.left_stick_y != 0) {
            robot.liftMotor.setPower(gamepad2.left_stick_y);
        }
    }
}

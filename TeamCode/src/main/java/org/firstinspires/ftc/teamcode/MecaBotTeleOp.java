package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "MecaBotTeleOp")
public class MecaBotTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    private boolean     bIgnoreLiftStops = false;

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
            setup();
            drive();
            lift();
            intake();
            bumper();
            sidearm();
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }

    private void setup() {
        /*
         * Use special key combinations to toggle override controls
         */
        // The X button on both gamepads allows the lift stops to be overridden
        // This is necessary and useful when Robot is powered off with lift still raised high
        // and next power up initializes the lift bottom stop in that raised position
        if ((gamepad1.x) && (gamepad2.x)) {
            bIgnoreLiftStops = true;
        }
        // The Y button on both gamepads enforces the lift stops (top and bottom) as normal function
        if ((gamepad1.y) && (gamepad2.y)) {
            bIgnoreLiftStops = false;
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
            // forwared press on joystick is negative, backward press (towards human) is positivek8
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

        // The game pad joystick is negative when pushed forwards or upwards.
        // We want this direction to raise the lift upwards, therefore flip the sign to positive.
        float power = -gamepad2.left_stick_y;

        // if lift stops are being ignored then simply apply the joystick power to the motor
        if (bIgnoreLiftStops) {
            robot.liftMotor.setPower(power);
        }
        // lift upwards direction
        else if (power > 0 && robot.liftMotor.getCurrentPosition() < robot.LIFT_TOP) {
            robot.liftMotor.setPower(power);
        }
        // lift downwards direction
        else if (power < 0 && robot.liftMotor.getCurrentPosition() > robot.LIFT_BOTTOM) {
            robot.liftMotor.setPower(power);
        }
        // stop the lift movement
        else {
            robot.liftMotor.setPower(0);
        }

        telemetry.addData(">", "GP2 left joystick = " + gamepad2.left_stick_y);
        telemetry.addData(">", "Lift tick count = " + robot.liftMotor.getCurrentPosition());
        //robot.liftServo.setPosition(robot.liftServo.getPosition() + (gamepad2.right_stick_y / 20));

        /*
         * Lift Arm control
         */
        if (gamepad2.right_stick_y != 0) {
            double newpos;
            // forwared press on joystick is negative, backward press (towards human) is positive
            // If operator joystick is pushed forwards/upwards, move the lift arm outside the robot
            if (gamepad2.right_stick_y < 0) {
                newpos = robot.liftServo.getPosition() - robot.ARM_STEP;
            } else {
                newpos = robot.liftServo.getPosition() + robot.ARM_STEP;
            }
            newpos = Range.clip(newpos, robot.ARM_OUTSIDE, robot.ARM_INSIDE);
            robot.liftServo.setPosition(newpos);
            telemetry.addData(">", "lift servo new pos %5.2f", newpos);
            telemetry.addData(">", "right joystick pushed %5.2f", gamepad2.right_stick_y);
        }

        //robot.clawRotate.setPosition(robot.clawRotate.getPosition() + (gamepad2.right_trigger / 20) - (gamepad2.left_trigger / 20));

        // If operator right trigger is pressed, rotate claw to Stone pickup position inside robot
        if (gamepad2.x) {
            robot.clawRotate.setPosition(robot.CLAW_PARALLEL);
            telemetry.addData(">", "right trigger pushed %5.2f", gamepad2.right_trigger);

        }
        // If operator left trigger is pressed, rotate claw perpendicular to Stone pickup position
        else if (gamepad2.y) {
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

        if (gamepad2.right_trigger > 0) { // intake is sucking the stone in to the robot
            robot.leftIntake.setPower(-gamepad2.right_trigger);
            robot.rightIntake.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0) { // intake is ejecting the stone out of the robot
            robot.leftIntake.setPower(gamepad2.left_trigger);
            robot.rightIntake.setPower(gamepad2.left_trigger);
        }
        else { // stop intake motors
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

    }

    public void bumper() {
        if (gamepad2.b) {
            robot.bumperServo.setPosition(robot.BUMPER_UP); // bumper up and free
        }
        else if (gamepad2.a) {
            robot.bumperServo.setPosition(robot.BUMPER_DOWN); // bumper down to engage the foundation
        }
    }

    public void sidearm() {
        if (gamepad1.b) {
            robot.sideArmServo.setPosition(robot.SIDEARM_UP); // side arm up and free
        }
        else if (gamepad1.a) {
            robot.sideArmServo.setPosition(robot.SIDEARM_DOWN); // side arm down to engage the stone
        }
    }
}

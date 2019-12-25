package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.MecaBot;


@TeleOp(name = "MecaBotTeleOp", group="QT")
public class MecaBotTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double TURN_FACTOR =   0.6;    // slow down turning speed
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
        //if we want to move sideways (MECANUM)
        if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
            robot.driveMecanum(gamepad1.left_stick_x);
        }
        // normal tank movement
        else {
            // forwared press on joystick is negative, backward press (towards human) is positive, reverse sign
            // right press on joystick is positive, left press is negative
            robot.driveTank(-gamepad1.left_stick_y, gamepad1.right_stick_x * TURN_FACTOR);
        }
    }

    public void lift() {

        // The game pad joystick is negative when pushed forwards or upwards.
        // We want this direction to raise the lift upwards, therefore flip the sign to positive.
        float power = -gamepad2.left_stick_y;
        // current position of lift can be positive or negative depending on FORWARD or REVERSE rotation setting
        // The reference position (lift collapsed or at bottom) = 0 encoder count, initialized at power up
        double pos = robot.liftMotor.getCurrentPosition();

        // if lift stops are being ignored then simply apply the joystick power to the motor
        if (bIgnoreLiftStops) {
            robot.liftMotor.setPower(power);
        }
        // move lift upwards direction but respect the stop to avoid breaking string
        else if (power > 0 && pos < robot.LIFT_TOP) {
            robot.liftMotor.setPower(power);
        }
        // move lift downwards direction but respect the stop to avoid winding string in opposite direction on the spool
        else if (power < 0 && pos > robot.LIFT_BOTTOM) {
            robot.liftMotor.setPower(power);
        }
        // stop the lift movement
        else {
            robot.liftMotor.setPower(0);
        }

        telemetry.addData(">", "GP2 left joystick = " + gamepad2.left_stick_y);
        telemetry.addData(">", "Lift tick count = " + pos);
        //robot.liftServo.setPosition(robot.liftServo.getPosition() + (gamepad2.right_stick_y / 20));

        /*
         * Lift Arm control
         */
        if (gamepad2.right_stick_y != 0) {
            double newpos;
            // forwared press on joystick is negative, backward press (towards human) is positive
            // If operator joystick is pushed forwards/upwards, move the lift arm inside the robot
            if (gamepad2.right_stick_y < 0) {
                newpos = robot.liftServo.getPosition() - MecaBot.ARM_STEP; // outside to inside is counter-clockwise
            } else {
                newpos = robot.liftServo.getPosition() + MecaBot.ARM_STEP; // inside to outside is clockwise
            }
            newpos = Range.clip(newpos, Servo.MIN_POSITION, Servo.MAX_POSITION);
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
            robot.grabStoneWithClaw(); // right is grab the stone, claw closed
            telemetry.addData(">", "right bumper pushed");
        }
        else if (gamepad2.left_bumper) {
            robot.releaseStoneWithClaw(); // left is release the stone, claw open
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
            robot.releaseFoundation();
        }
        else if (gamepad2.a) {
            robot.grabFoundation();
        }
    }

    public void sidearm() {
        if (gamepad1.a) {
            robot.releaseStoneWithSidearm();
        }
        else if (gamepad1.b) {
            robot.grabStoneWithSidearm();
        }
    }
}

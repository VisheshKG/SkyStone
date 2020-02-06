package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;


@TeleOp(name = "Skystone TeleOp", group="QT")
public class SkystoneTeleOp extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double TURN_FACTOR =   0.6;    // slow down turning speed
    private boolean     bIgnoreLiftStops = false;

    /* Declare OpMode members. */
    MecaBot     robot = new MecaBot();   // Use a MecaBot's hardware
    MecaBotMove nav;
    OdometryGlobalPosition globalPosition;

    // record position that we need to return to repeatedly
    double xpos, ypos, tpos;
    boolean autoDriving = false;
    double speedMultiplier = MecaBotMove.DRIVE_SPEED_MAX;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        telemetry.addData(">", "Hardware initialized");

        nav = new MecaBotMove(this, robot);
        globalPosition = nav.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Waiting for Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        nav.startOdometry();

        telemetry.addLine("Encoder ")
                .addData("Lift", "%d", new Func<Integer>() {
                    @Override public Integer value() {
                        return robot.getLiftCurrentPosition();
                    }
                })
                .addData("Arm", "%d", new Func<Integer>() {
                    @Override public Integer value() {
                        return robot.liftArmMotor.getCurrentPosition();
                    }
                });
        telemetry.addLine("Global Position ")
                .addData("X", "%3.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Angle", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        telemetry.addLine("Move ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return nav.getMovementStatus();
                    }
                });

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setup();
            autodrive();
            operdrive();
            lift();
            intake();
            bumper();
            capstone();
            telemetry.update();
            idle();
        }

        //Stop the thread
        nav.stopOdometry();

        // all done, please exit

    }

    private void setup() {
        /*
         * Use special key combinations to toggle override controls
         */
        // Allow the lift stops to be overridden
        // This is necessary and useful when Robot is powered off with lift still raised high
        // and next power up initializes the lift bottom stop in that raised position
        if ((gamepad1.x) && (gamepad2.x)) {
            bIgnoreLiftStops = true;
        }
        // Enforces the lift stops (top and bottom) as normal function
        if ((gamepad1.y) && (gamepad2.y)) {
            bIgnoreLiftStops = false;
        }
        // Toggle which face of the Robot is front for driving
        if ((gamepad1.dpad_up) || (gamepad1.dpad_right)) {  // dpad_right means green INTAKE wheels is front of robot
            robot.setFrontIntake();
        } else if ((gamepad1.dpad_down) || (gamepad1.dpad_left)) {
            robot.setFrontLiftarm(); // dpad_left means Liftarm face is front of robot
        }
        if ((gamepad1.x) && (!gamepad2.x)) {
            xpos = globalPosition.getXinches();
            ypos = globalPosition.getYinches();
            tpos = globalPosition.getOrientationDegrees();
            telemetry.addData("Locked Position", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
        }
        if ((gamepad1.y) && (!gamepad2.y)){
            robot.setFrontLiftarm();
            autoDriving = true;
        }
        //update speedMultiplier
        if (gamepad1.right_bumper) {
            speedMultiplier = MecaBotMove.DRIVE_SPEED_MAX;
            robot.setFastBlue();
            // as a dual action of this button stop autodriving
            autoDriving = false;
        }
        else if (gamepad1.left_bumper) {
            speedMultiplier = MecaBotMove.DRIVE_SPEED_DEFAULT;
            robot.setSlowBlue();
            // as a dual action of this button stop autodriving
            autoDriving = false;
        }

    }

    public void autodrive() {
        if (autoDriving) {
            telemetry.addData("Driving Towards", "X %2.2f | Y %2.2f | Angle %3.2f", xpos, ypos, tpos);
            double distance = nav.goTowardsPosition(xpos, ypos, MecaBotMove.DRIVE_SPEED_DEFAULT, true);
            if (distance < MecaBotMove.DIST_MARGIN) { // we have reached
                autoDriving = false;
            }
        }
    }

    public void operdrive() {
        // if joystick is inactive brakes will be applied during autodrive() therefore don't go in
        if (autoDriving) {
            return;
        }

        double power, turn;

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.

        // when we want to move sideways (MECANUM)
        if (gamepad1.left_trigger > 0) {
            power = -gamepad1.left_trigger; // negative power to move LEFT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Left ", "%.2f", power);
        }
        else if (gamepad1.right_trigger > 0) {
            power = gamepad1.right_trigger; // positive power to move RIGHT
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.30 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.25 + (0.75 * power * power)) * speedMultiplier;
            robot.driveMecanum(power);
            telemetry.addData("Mecanum Right  ", "%.2f", power);
        }
        // normal tank movement
        else {  // when joystick is inactive, this applies brakes, be careful to avoid during autodrive
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.
            power = -gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;

            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.25 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.2 + (0.8 * power * power));
            // OR
            // use this to scale the range without squaring the power value
            //power = Math.signum(power) * (0.25 + (0.75 * Math.abs(power)));
            // OR
            // use this to square the power while preserving the sign, without scaling the range
            //power *= Math.abs(power);

            // similarly for turn power, except also slow down by TURN_FACTOR
            turn = Math.signum(turn) * (0.1 + (TURN_FACTOR * turn * turn));

            robot.driveTank(power, turn);
            telemetry.addData("Tank Power", "Drive=%.2f Turn=%.2f", power, turn);
        }
    }

    public void lift() {

        //
        // Vertical Lift control
        //
        if (gamepad2.left_stick_y != 0) {
            // The game pad joystick is negative when pushed forwards or upwards.
            // Our lift motor raises the lift when positive power is applied
            // To raise the lift with a forward/upwards push of joystick, flip the sign
            double power = -gamepad2.left_stick_y;
            // Square the number but retain the sign to convert to logarithmic scale
            // scale the range to 0.15 <= abs(power) <= 1.0 and preserve the sign
            power = Math.signum(power) * (0.15 + (0.85 * power * power));
            // OR
            // use this to scale the range without squaring the power value
            //power = Math.signum(power) * (0.15 + (0.85 * Math.abs(power)));
            // OR
            // use this to square the power while preserving the sign, without scaling the range
            //power *= Math.abs(power);

            // current position of lift can be positive or negative depending on FORWARD or REVERSE rotation setting
            // The reference position (lift collapsed or at bottom) = 0 encoder count, initialized at power up
            int pos = robot.getLiftCurrentPosition();

            // if lift stops are being ignored then simply apply the joystick power to the motor
            if (bIgnoreLiftStops) {
                robot.liftMotor.setPower(power);
                telemetry.addData("LIFT ", "at %3d, IGNORING STOPS", pos);
            }
            // move lift upwards direction but respect the stop to avoid breaking string
            else if (power > 0 && pos < robot.LIFT_TOP) {
                robot.liftMotor.setPower(power);
                telemetry.addData("Lift Up", "%.2f", power);
            }
            // move lift downwards direction but respect the stop to avoid winding string in opposite direction on the spool
            else if (power < 0 && pos > robot.LIFT_BOTTOM) {
                robot.liftMotor.setPower(power);
                telemetry.addData("Lift Down", "%.2f", power);
            }
            else {
                robot.stopLift();
            }
        }
        else if (gamepad2.dpad_up) {
            nav.moveLiftUp();
        }
        else if (gamepad2.dpad_down) {
            nav.moveLiftDown();
        }
        else {
            robot.stopLift();
        }

        //
        // Lift Arm control
        //
        if (gamepad2.right_stick_y != 0) {
            // forward press on joystick is negative, backward press (towards human) is positive
            // Positive power moves the lift arm outside, negative power moves the lift arm inside the robot
            // This works out for us, no need to flip the sign
            double power = gamepad2.right_stick_y;
            // Square the number but retain the sign, to convert to logarithmic scale
            power *= Math.abs(power);

            int pos = robot.liftArmMotor.getCurrentPosition();
            // if lift stops are being ignored then simply apply the joystick power to the motor
            if (bIgnoreLiftStops) {
                robot.liftArmMotor.setPower(power);
                telemetry.addData("ARM", "at %3d, IGNORING STOPS", pos);
            }
            // move lift arm outside direction but respect the stop to avoid breaking string
            else if (power > 0 && pos < MecaBot.ARM_OUTSIDE) {
                robot.liftArmMotor.setPower(power);
                telemetry.addData("Arm Out", "%.2f", power);
            }
            // move lift arm inside direction but respect the stop to avoid winding string in opposite direction on the spool
            else if (power < 0 && pos > MecaBot.ARM_INSIDE) {
                robot.liftArmMotor.setPower(power);
                telemetry.addData("Arm In", "%.2f", power);
            }
            else {
                robot.stopLiftArm();
            }
        }
        else if (gamepad2.dpad_left) {
            //nav.moveLiftArmOutside();
            robot.rotateClawOutside();
        }
        else if (gamepad2.dpad_right) {
            //nav.moveLiftArmInside();
            robot.rotateClawInside();
        }
        else {
            robot.stopLiftArm();
        }

        //
        // Claw control for pickup and delivery of stone
        //
        // temporarily override the x and y buttons to move the lift arm and not the claw
        // this is because the lift arm encoder is drifting causing trouble with software stops
        // so we need some button to reset the encoder count when lift arm is at resting position
        if (gamepad2.x) {
            //nav.moveLiftArmInside();
            robot.liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad2.y) {
            //robot.liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            nav.moveLiftArmOutside();
        }
        if (gamepad2.right_bumper) {
            robot.grabStoneWithClaw(); // right is grab the stone, claw closed
        }
        else if (gamepad2.left_bumper) {
            robot.releaseStoneWithClaw(); // left is release the stone, claw open
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

    public void capstone() {
        if (gamepad1.a) {
            robot.holdCapstone();
        }
        else if (gamepad1.b) {
            robot.releaseCapstone();
        }
    }

}

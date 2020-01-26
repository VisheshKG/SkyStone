package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    double speedMultiplier = 1.0;

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
                        return robot.liftMotor.getCurrentPosition();
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
            sleep(CYCLE_MS);
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
        if (autoDriving) {
            return;
        }

        //update speedMultiplier
        if (gamepad1.right_bumper) {
            speedMultiplier = 1.0;
            robot.setFastBlue();
        }
        else if (gamepad1.left_bumper) {
            speedMultiplier = 0.66;
            robot.setSlowBlue();
        }

        double drive_x = gamepad1.left_stick_x;
        double drive_y = gamepad1.left_stick_y;
        double turn_x = gamepad1.right_stick_x;

        // square the joystick values to change from linear to logarithmic scale
        // this allows more movement of joystick for less movement of robot, thus more precision at lower speeds
        // at the expense of some loss of precision at higher speeds, where it is not required.

        //if we want to move sideways (MECANUM)
        if (Math.abs(drive_x) > Math.abs(drive_y)) {
            robot.driveMecanum(drive_x * Math.abs(drive_x) * speedMultiplier);
        }
        // normal tank movement
        else{  // only if joystick is active, otherwise brakes are applied during autodrive()
            // forward press on joystick is negative, backward press (towards human) is positive
            // right press on joystick is positive value, left press is negative value
            // reverse sign of joystick values to match the expected sign in driveTank() method.
            robot.driveTank(-drive_y * Math.abs(drive_y) * speedMultiplier,
                    -turn_x * Math.abs(turn_x) * speedMultiplier * TURN_FACTOR);
        }
    }

    public void lift() {

        /*
         * Vertical Lift control
         */
        if (gamepad2.left_stick_y != 0) {
            // The game pad joystick is negative when pushed forwards or upwards.
            // We want this direction to raise the lift upwards, therefore flip the sign.
            double power = -gamepad2.left_stick_y;
            // Square the number but retain the sign - to convert to logarithmic scale
            power *= Math.abs(power);

            // current position of lift can be positive or negative depending on FORWARD or REVERSE rotation setting
            // The reference position (lift collapsed or at bottom) = 0 encoder count, initialized at power up
            int pos = robot.liftMotor.getCurrentPosition();

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
        }
        // stop the lift movement
        else {
            robot.liftMotor.setPower(0);
        }

        /*
         * Lift Arm control
         */
        if (gamepad2.right_stick_y != 0) {
            // forward press on joystick is negative, backward press (towards human) is positive
            // We want this direction to raise the arm outside, therefore flip the sign to positive.
            double power = -gamepad2.right_stick_y;
            // Square the number but retain the sign - to convert to logarithmic scale
            power *= Math.abs(power);

            int pos = robot.liftArmMotor.getCurrentPosition();
            // If operator joystick is pushed forwards/upwards, move the lift arm inside the robot
            // if lift stops are being ignored then simply apply the joystick power to the motor
            if (bIgnoreLiftStops) {
                robot.liftArmMotor.setPower(power);
            }
            // move lift arm outside direction but respect the stop to avoid breaking string
            else if (power > 0 && pos < robot.ARM_OUTSIDE) {
                robot.liftArmMotor.setPower(power);
            }
            // move lift arm inside direction but respect the stop to avoid winding string in opposite direction on the spool
            else if (power < 0 && pos > robot.ARM_INSIDE) {
                robot.liftArmMotor.setPower(power);
            }
        }
        else {
            robot.liftArmMotor.setPower(0);
        }

        if (gamepad2.x) {
            robot.rotateClawInside();
        }
        else if (gamepad2.y) {
            robot.rotateClawOutside();
        }

        if (gamepad2.right_bumper) {
            robot.grabStoneWithClaw(); // right is grab the stone, claw closed
        }
        else if (gamepad2.left_bumper) {
            robot.releaseStoneWithClaw(); // left is release the stone, claw open
        }

        if (gamepad2.dpad_right) {
            robot.moveLiftArmOutside();
        }
        else if (gamepad2.dpad_left) {
            robot.moveLiftArmInside();
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
            robot.resetCapstoneClips();
        }
        else if (gamepad1.b) {
            robot.dropCapstoneClips();
        }
    }

}

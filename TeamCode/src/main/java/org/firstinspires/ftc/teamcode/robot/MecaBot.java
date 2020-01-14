/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Mecabot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note: All names are camel case without spaces.
 * Note: The names can be found in the init() method where literal strings are used to initialize hardware.
 *
 */
public class MecaBot {
    // drive train motors
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    // odometry encoder wheels
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor horizontalEncoder = null;

    // [skystone] front intake (green compliant wheels for skystone blocks)
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;

    // [skystone] lift and arm and claw for picking and delivering stones (blocks)
    public DcMotor liftMotor = null;
    public Servo liftServo = null;
    public Servo clawRotate = null;
    public Servo clawGrab = null;
    public Servo sideArmServo = null;

    // [skystone] foundation bumper clamps on rear of the robot
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    // [skystone] color sensors used for detecting skystone vs normal stone in quarry
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;


    //constants here
    public static final double LENGTH = 17.0;
    public static final double WIDTH = 18.0;
    public static final double HALF_WIDTH = WIDTH / 2;

    public static final double LIFT_TOP = 11400;
    public static final double LIFT_BOTTOM = 0;
    public static final double ARM_INSIDE = 0.0;
    public static final double ARM_OUTSIDE = 0.3;
    public static final double ARM_STEP = 0.05;
    public static final double CLAW_PARALLEL = Servo.MAX_POSITION;
    public static final double CLAW_PERPENDICULAR = Servo.MIN_POSITION;
    public static final double CLAW_OPEN = Servo.MAX_POSITION;
    public static final double CLAW_CLOSE = Servo.MIN_POSITION;
    public static final double RT_BUMPER_UP = Servo.MAX_POSITION;
    public static final double RT_BUMPER_DOWN = Servo.MIN_POSITION;
    public static final double LT_BUMPER_UP = Servo.MIN_POSITION;
    public static final double LT_BUMPER_DOWN = Servo.MAX_POSITION;

    public static final double SIDEARM_UP = Servo.MAX_POSITION;
    public static final double SIDEARM_DOWN = Servo.MIN_POSITION;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    public static final double ODOMETRY_COUNT_PER_INCH = 242.552133272048492;  // FTC Team 13345 MecaBot encoder has 1440 ticks per rotation, wheel has 48mm diameter
    // (1440 * MM_PER_INCH) / ( Math.PI * 48)

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public MecaBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /* Define and Initialize Motors and Servos */

        // Drivetrain motors
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetDriveEncoder();
        // Set all drivetrain motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake
        leftIntake = hwMap.get(DcMotor.class, "leftIntake");
        rightIntake = hwMap.get(DcMotor.class, "rightIntake");
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry encoders
        leftEncoder = leftIntake;   // we are using the encoder port of intake motor for odometry
        rightEncoder = rightIntake; // we are using the encoder port of intake motor for odometry
        horizontalEncoder = hwMap.dcMotor.get("horizontalEncoder");
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // We are not driving any motor using encoder, the odometry wheels are free to rotate, the encoders will still read correct values

        // Lift, arm and claw
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hwMap.get(Servo.class, "liftServo");
        clawRotate = hwMap.get(Servo.class, "clawRotate");
        clawGrab = hwMap.get(Servo.class, "clawGrab");
        sideArmServo = hwMap.get(Servo.class, "sideArmServo");

        // foundation bumper clamps
        leftClamp = hwMap.get(Servo.class, "leftClamp");
        rightClamp = hwMap.get(Servo.class, "rightClamp");

        // color sensors
        leftColorSensor = hwMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hwMap.get(ColorSensor.class, "rightColorSensor");

        // Set all motors to zero power
        stopDriving();
        stopIntake();
        stopLift();

        // set all servos to their resting position
        moveLiftArmInside();
        rotateClawInside();
        releaseStoneWithClaw();
        releaseStoneWithSidearm();
        releaseFoundation();
    }

    public OdometryGlobalPosition initOdometry() throws IllegalStateException {

        if ((leftEncoder == null) || (rightEncoder == null) || (horizontalEncoder == null)) {
            throw new IllegalStateException("Mecabot hardware must be initialized before Odometry.");
        }

        //Create and start GlobalPosition thread to constantly update the global position coordinates.
        OdometryGlobalPosition globalPosition = new OdometryGlobalPosition(leftEncoder, rightEncoder, horizontalEncoder, ODOMETRY_COUNT_PER_INCH, 75);

        // Set direction of odometry encoders.
        // PLEASE UPDATE THESE VALUES TO MATCH YOUR ROBOT HARDWARE *AND* the DCMOTOR DIRECTION (FORWARD/REVERSE) CONFIGURATION
        // Left encoder value, robot forward movement should produce positive encoder count
        globalPosition.reverseLeftEncoder();
        // Right encoder value, robot forward movement should produce positive encoder count
        globalPosition.reverseRightEncoder();
        // Horizontal encoder value, robot right sideways movement should produce positive encoder count values
        globalPosition.reverseNormalEncoder();

        return globalPosition;
    }

    /*
     * Driving movement methods
     */
    public void setTargetPosition(int leftFront, int leftBack, int rightFront, int rightBack) {
        leftFrontDrive.setTargetPosition(leftFront);
        leftBackDrive.setTargetPosition(leftBack);
        rightFrontDrive.setTargetPosition(rightFront);
        rightBackDrive.setTargetPosition(rightBack);
    }

    public void setDriveMode(DcMotor.RunMode runMode) {
        leftBackDrive.setMode(runMode);
        leftFrontDrive.setMode(runMode);
        rightBackDrive.setMode(runMode);
        rightFrontDrive.setMode(runMode);

    }
    public void resetDriveEncoder() {
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveStraight(double speed) {

        speed = Range.clip( speed, -1.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
    }

    public void stopDriving() {
        this.driveStraight(0);
    }

    /**
     * Tank mode drives in forward (or backward) direction only, no mecanum sideways movement
     * The turns are achieved by speed differential between left and right side wheels
     * Sign value of speed parameters controls direction. +ve driveSpeed means forward and
     * +ve turnSpeed means turn left (since turn angle theta is +ve when counter clockwise from X-axis)
     *
     * @param driveSpeed    forward speed specified within range [-1.0, 1.0]
     * @param turnSpeed     turning speed specified within range [-1.0, 1.0]
     */
    public void driveTank(double driveSpeed, double turnSpeed) {

        driveSpeed = Range.clip(driveSpeed, -1.0, 1.0);

        // basic forward/backwards, run motors at drive speed, negative speed is reverse direction
        double leftFront = driveSpeed;
        double leftBack = driveSpeed;
        double rightFront = driveSpeed;
        double rightBack = driveSpeed;

        // left turn is positive turnSpeed, right turn is negative turnSpeed
        // to turn left, add turnSpeed to right motors, subtract from left motors
        // to turn right, same code applies, negative values cause right turn automatically
        turnSpeed = Range.clip(turnSpeed, -1.0, 1.0);
        leftFront -= turnSpeed;
        leftBack -= turnSpeed;
        rightFront += turnSpeed;
        rightBack += turnSpeed;

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }

    public void driveMecanum(double sideSpeed) {
        // if we want to move right sideways, sideSpeed value is positive
        // right inside
        double rightFront = -sideSpeed;
        double rightBack = +sideSpeed;

        // left outside
        double leftFront = +sideSpeed;
        double leftBack = -sideSpeed;

        // if we want to move left, its same code as above, sideSpeed value is negative

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }

    public void driveWheels(double leftFront, double leftBack, double rightFront, double rightBack) {
        // find the highest power motor and divide all motors by that to preserve the ratio
        // while also keeping the maximum power at 1
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        //set drive train motor's power to the values calculated
        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }

    /*
     * Intake operation methods
     */
    public void runIntake(double speed) {
        leftIntake.setPower(speed);
        rightIntake.setPower(speed);
    }
    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    /*
     * Lift, arm and claw operation methods
     */
    public void stopLift() {
        liftMotor.setPower(0);
    }
    public void moveLiftArmInside() {
        liftServo.setPosition(ARM_INSIDE);
    }
    public void moveLiftArmOutside() {
        liftServo.setPosition(ARM_OUTSIDE);
    }
    public void rotateClawInside() {
        clawRotate.setPosition(CLAW_PARALLEL);
    }
    public void rotateClawOutside() {
        clawRotate.setPosition(CLAW_PERPENDICULAR);
    }
    public void grabStoneWithClaw() {
        clawGrab.setPosition(CLAW_CLOSE);
    }
    public void releaseStoneWithClaw() {
        clawGrab.setPosition(CLAW_OPEN);
    }

    /*
     * Foundation bumper clamps operataion methods
     */
    public void grabFoundation() {
        leftClamp.setPosition(LT_BUMPER_DOWN); // clamp down to engage the foundation
        rightClamp.setPosition(RT_BUMPER_DOWN);
    }
    public void releaseFoundation() {
        leftClamp.setPosition(LT_BUMPER_UP); // clamp up to release the foundation
        rightClamp.setPosition(RT_BUMPER_UP);
    }
    public void grabStoneWithSidearm() {
        sideArmServo.setPosition(SIDEARM_DOWN); // side arm down to engage the stone
    }
    public void releaseStoneWithSidearm() {
        sideArmServo.setPosition(SIDEARM_UP); // side arm up and free
    }
}


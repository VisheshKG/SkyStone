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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MecaBot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    public Servo liftServo = null;
    public Servo clawRotate = null;
    public Servo clawGrab = null;
    public Servo bumperServo = null;
    public Servo sideArmServo = null;

    //constants here
    public static final double LIFT_TOP = 4950;
    public static final double LIFT_BOTTOM = 0;
    public static final double ARM_INSIDE = Servo.MAX_POSITION;
    public static final double ARM_OUTSIDE = Servo.MIN_POSITION;
    public static final double ARM_STEP = 0.04;
    public static final double CLAW_PARALLEL = 0.92;
    public static final double CLAW_PERPENDICULAR = 0.60;
    public static final double CLAW_OPEN = 0.45;
    public static final double CLAW_CLOSE = 0.40;
    public static final double BUMPER_UP = Servo.MAX_POSITION;
    public static final double BUMPER_DOWN = Servo.MIN_POSITION;
    public static final double SIDEARM_UP = Servo.MAX_POSITION;
    public static final double SIDEARM_DOWN = Servo.MIN_POSITION;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public MecaBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        leftIntake = hwMap.get(DcMotor.class, "leftIntake");
        rightIntake = hwMap.get(DcMotor.class, "rightIntake");
        liftServo = hwMap.get(Servo.class, "liftServo");
        clawRotate = hwMap.get(Servo.class, "clawRotate");
        clawGrab = hwMap.get(Servo.class, "clawGrab");
        bumperServo = hwMap.get(Servo.class, "bumperServo");
        sideArmServo = hwMap.get(Servo.class, "sideArmServo");

        // Set motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        liftMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        // set all servos to their resting position
        liftServo.setPosition(ARM_INSIDE);
        clawRotate.setPosition(CLAW_PARALLEL);
        clawGrab.setPosition(CLAW_OPEN);
        bumperServo.setPosition(BUMPER_UP);
        sideArmServo.setPosition(SIDEARM_UP);

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

    public void driveForwardBackStop(double speed) {

        speed = Range.clip( speed, -1.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
    }

    public void driveStraight(double speed) {
        this.driveForwardBackStop(speed);
    }

    public void stopDriving() {
        this.driveForwardBackStop(0);
    }

    public void driveTank(double driveSpeed, double turnSpeed) {

        driveSpeed = Range.clip(driveSpeed, -1.0, 1.0);

        // basic forward/backwards, run motors at drive speed, negative speed is reverse direction
        double leftFront = driveSpeed;
        double leftBack = driveSpeed;
        double rightFront = driveSpeed;
        double rightBack = driveSpeed;

        // right turn is positive turnSpeed, left turn is negative turnSpeed
        // to turn right, add turnSpeed to left motors, subtract from right motors
        // to turn left, same code applies, negative values cause left turn automatically
        turnSpeed = Range.clip(turnSpeed, -1.0, 1.0);
        leftFront += turnSpeed;
        leftBack += turnSpeed;
        rightFront -= turnSpeed;
        rightBack -= turnSpeed;

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

    public void grabFoundation() {
        bumperServo.setPosition(BUMPER_DOWN); // bumper down to engage the foundation
    }
    public void releaseFoundation() {
        bumperServo.setPosition(BUMPER_UP); // bumper down to engage the foundation
    }
    public void grabStoneWithSidearm() {
        sideArmServo.setPosition(SIDEARM_DOWN); // side arm down to engage the stone
    }
    public void releaseStoneWithSidearm() {
        sideArmServo.setPosition(SIDEARM_UP); // side arm up and free
    }
}


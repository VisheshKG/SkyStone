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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;

import static java.lang.Thread.sleep;

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
    public DcMotor liftArmMotor = null;
    public Servo clawRotate = null;
    public Servo clawGrab = null;
    public Servo capstoneServo = null;

    // [skystone] foundation bumper clamps on rear of the robot
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    // [skystone] color sensors used for detecting skystone vs normal stone in quarry
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;
    public ColorSensor blockColorSensor = null;

    // Lights control
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern = null;

    // Robot front and rear can be flipped for driving purposes
    // Define enum constant for whether INTAKE or LIFT is Front of the robot (and other one is Rear)
    static enum DIRECTION {INTAKE, LIFTARM};
    DIRECTION frontFace;

    //constants here
    public static final double LENGTH = 17.0;
    public static final double WIDTH = 18.0;
    public static final double HALF_WIDTH = WIDTH / 2;

    // for goBilda 5202 series 26.9:1 gear ratio motor, encoder counts per rotation = 753.2
    // public static final int    LIFT_TOP = 6342;
    // for goBilda 5202 series 50.9:1 gear ratio motor, encoder counts per rotation = 1425.2
    public static final int    LIFT_TOP = 12000;
    public static final int    LIFT_BOTTOM = 50;        // Motor overshoots when software stop kicks in, allow some margin to ZERO position stops
    public static final int    ARM_INSIDE = 10;         // Motor overshoots when software stop kicks in, allow some margin to ZERO position stops
    public static final int    ARM_OUTSIDE = 400;
    public static final double CLAW_INSIDE = Servo.MAX_POSITION;
    public static final double CLAW_OUTSIDE = 0.20;     // adjustment to make the stone square with the robot and foundation
    public static final double CLAW_OPEN = Servo.MAX_POSITION;
    public static final double CLAW_CLOSE = Servo.MIN_POSITION;
    public static final double RT_BUMPER_UP = Servo.MAX_POSITION;
    public static final double RT_BUMPER_DOWN = Servo.MIN_POSITION;
    public static final double LT_BUMPER_UP = Servo.MIN_POSITION;
    public static final double LT_BUMPER_DOWN = Servo.MAX_POSITION;
    public static final double CAP_LOCKED = Servo.MAX_POSITION;
    public static final double CAP_RELEASED = Servo.MIN_POSITION;

    public static final double MOTOR_STOP_SPEED = 0.0;
    public static final double LIFT_DEF_SPEED = 1.0;
    public static final double ARM_DEF_SPEED = 1.0;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    public static final double ODOMETRY_COUNT_PER_INCH = 242.552133272048492;  // FTC Team 13345 MecaBot encoder has 1440 ticks per rotation, wheel has 48mm diameter
    // (1440 * MM_PER_INCH) / ( Math.PI * 48)

    /* local OpMode members. */
    // The hardware map obtained from OpMode
    HardwareMap hwMap;

    // The IMU sensor object
    BNO055IMU imu;

    /* Constructor */
    public MecaBot() {

        frontFace = DIRECTION.INTAKE;
    }

    /*
     * Robot front facing direction toggle methods. Robot FRONT direction can be flipped.
     * This is important to understand, to avoid unexpected behavior.
     * When Robot front direction is toggled from INTAKE to LIFTARM, the direction change is
     * achieved by changing only 2 underlying methods, all other code is oblivious of this.
     * @see MecaBot#setTargetPosition()
     * @see MecaBot#driveWheels()
     *
     * MOST IMPORTANT: All driving and move methods must call one of the above methods,
     * and must NOT set drivetrain motor power directly. Specifically the methods
     * @see MecaBot#SetDrivePower()  do NOT handle frontFace flipping.
     * They are used for non-directional movement, such as gyro rotation.
     */
    public DIRECTION frontDirection() {
        return frontFace;
    }
    public boolean isFrontIntake() {
        return (frontFace == DIRECTION.INTAKE);
    }
    public boolean isFrontLiftarm() {
        return (frontFace == DIRECTION.LIFTARM);
    }
    public void setFrontIntake() {
        frontFace = DIRECTION.INTAKE;
        setLightGreen();
    }
    public void setFrontLiftarm() {
        frontFace = DIRECTION.LIFTARM;
        setLightRed();
    }
    public String getFrontDirection() {
        return ((frontFace == DIRECTION.INTAKE) ? "INTAKE" : "LIFTARM");
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
        horizontalEncoder = leftBackDrive; // we are using the encoder port of left Back Drivetrain motor

        // Lift, arm and claw
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        //liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftArmMotor = hwMap.get(DcMotor.class, "liftArmMotor");
        liftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawRotate = hwMap.get(Servo.class, "clawRotate");
        clawGrab = hwMap.get(Servo.class, "clawGrab");
        capstoneServo = hwMap.get(Servo.class, "capstoneServo");

        // foundation bumper clamps
        leftClamp = hwMap.get(Servo.class, "leftClamp");
        rightClamp = hwMap.get(Servo.class, "rightClamp");

        // color sensors
        leftColorSensor = hwMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hwMap.get(ColorSensor.class, "rightColorSensor");
        blockColorSensor = hwMap.get(ColorSensor.class, "blockColorSensor");

        // lights
        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        // Set all motors to zero power
        stopDriving();
        stopIntake();
        stopLift();
        stopLiftArm();

        // set all servos to their resting position
        rotateClawInside();
        releaseStoneWithClaw();
        holdCapstone();
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

    public void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /*
     * Driving movement methods
     */

    /**
     * Set encoder position for each wheel in preparation for a RUN_USING_ENCODERS movement
     * This method DOES take into account the frontFace of the robot, whether Intake or Liftarm
     * and corresponding to robot front direction, sets the target encoder value on the wheels
     * Note: {@link MecaBot#driveWheels(double, double, double, double)} also accounts for frontFace
     * Note: {@link MecaBot#setDrivePower(double)} does NOT consider frontFace of the robot, the most
     * common use of that method is after {@link MecaBot#setTargetPosition(int, int, int, int)} has been called
     * @param leftFront  Left front wheel target encoder count
     * @param leftBack   Left back wheel target encoder count
     * @param rightFront Right front wheel target encoder count
     * @param rightBack  Right back wheel target encoder count
     */
    public void setTargetPosition(int leftFront, int leftBack, int rightFront, int rightBack) {
        if (isFrontIntake()) {
            leftFrontDrive.setTargetPosition(leftFront);
            leftBackDrive.setTargetPosition(leftBack);
            rightFrontDrive.setTargetPosition(rightFront);
            rightBackDrive.setTargetPosition(rightBack);
        }
        else { // isFrontLiftarm()
            leftFrontDrive.setTargetPosition(-rightBack);
            leftBackDrive.setTargetPosition(-rightFront);
            rightFrontDrive.setTargetPosition(-leftBack);
            rightBackDrive.setTargetPosition(-leftFront);
        }
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

    public void setDrivePower(double speed) {
        speed = Range.clip( speed, 0.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
    }

    public void setDrivePower(double leftSpeed, double rightSpeed) {
        leftSpeed = Range.clip( leftSpeed, 0.0, 1.0);
        rightSpeed = Range.clip( rightSpeed, 0.0, 1.0);
        leftFrontDrive.setPower(leftSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightBackDrive.setPower(rightSpeed);
    }

    public void stopDriving() {
        this.setDrivePower(0);
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

    /**
     * Drives the robot diagonally
     * Positive values of speed drive it forwards and right
     * Negative values of speed drive it forwards and left
     * @param speed
     */
    public void driveDiagonal(double speed) {
        double leftFront = 0;
        double leftBack = 0;
        double rightFront = 0;
        double rightBack = 0;

        if (speed > 0) {
            leftFront = speed;
            rightBack = speed;
        }
        else if (speed < 0) {
            leftBack = Math.abs(speed);
            rightFront = Math.abs(speed);
        }

        driveWheels(leftFront, leftBack, rightFront, rightBack);
    }


    /**
     * Set drive wheels power to specified values, which are normalized to -1.0 <= power <= 1.0 range
     * This method DOES take into account the frontFace of the robot, whether Intake or Liftarm
     * and corresponding to robot front direction, sets the power on front/back wheels consistent with frontFace
     * Note: {@link MecaBot#setTargetPosition(int, int, int, int)} also accounts for frontFace
     * Note: {@link MecaBot#setDrivePower(double)} does NOT consider frontFace of the robot, the most
     * common use of that method is after {@link MecaBot#setTargetPosition(int, int, int, int)} has been called
     * @param leftFront  Left front wheel target encoder count
     * @param leftBack   Left back wheel target encoder count
     * @param rightFront Right front wheel target encoder count
     * @param rightBack  Right back wheel target encoder count
     */
    public void driveWheels(double leftFront, double leftBack, double rightFront, double rightBack) {
        // find the highest power motor and divide all motors by that to preserve the ratio
        // while also keeping the maximum power at 1
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1.0) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        //set drive train motor's power to the values calculated
        if (isFrontIntake()) {
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);
        }
        else { // isFrontLiftarm()
            leftFrontDrive.setPower(-rightBack);
            leftBackDrive.setPower(-rightFront);
            rightFrontDrive.setPower(-leftBack);
            rightBackDrive.setPower(-leftFront);
        }
    }

    /*
     * Intake operation methods
     */
    public void runIntake(double speed) {
        // we need negative power for sucking in the stones
        if (speed > 0) {
            speed = -speed;
        }
        leftIntake.setPower(speed);
        rightIntake.setPower(speed);
    }
    public void stopIntake() {
        leftIntake.setPower(MOTOR_STOP_SPEED);
        rightIntake.setPower(MOTOR_STOP_SPEED);
    }
    public void runOuttake(double speed) {
        // we need positive power for ejecting out the stones
        if (speed < 0) {
            speed = +speed;
        }
        leftIntake.setPower(speed);
        rightIntake.setPower(speed);
    }
    public void stopOuttake() {
        leftIntake.setPower(MOTOR_STOP_SPEED);
        rightIntake.setPower(MOTOR_STOP_SPEED);
    }

    /*
     * Lift, arm and claw operation methods
     */
    public int getLiftCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void stopLift() {
        liftMotor.setPower(MOTOR_STOP_SPEED);
        // Get out of the RunMode RUN_TO_POSITION, so manual player control is possible
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLift(int position) {
        position = Range.clip(position, LIFT_BOTTOM, LIFT_TOP);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_DEF_SPEED);
    }

    public void stopLiftArm() {
        liftArmMotor.setPower(MOTOR_STOP_SPEED);
        // Get out of the RunMode RUN_TO_POSITION, so manual player control is possible
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLiftArm(int position) {
        position = Range.clip(position, ARM_INSIDE, ARM_OUTSIDE);
        liftArmMotor.setTargetPosition(position);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArmMotor.setPower(ARM_DEF_SPEED);
    }

    public void rotateClawInside() {
        clawRotate.setPosition(CLAW_INSIDE);
    }
    public void rotateClawOutside() {
        clawRotate.setPosition(CLAW_OUTSIDE);
    }
    public void grabStoneWithClaw() {
        clawGrab.setPosition(CLAW_CLOSE);
    }
    public void releaseStoneWithClaw() {
        clawGrab.setPosition(CLAW_OPEN);
    }

    /*
     * Foundation bumper clamps operation methods
     */
    public void grabFoundation() {
        leftClamp.setPosition(LT_BUMPER_DOWN); // clamp down to engage the foundation
        rightClamp.setPosition(RT_BUMPER_DOWN);
    }
    public void releaseFoundation() {
        leftClamp.setPosition(LT_BUMPER_UP); // clamp up to release the foundation
        rightClamp.setPosition(RT_BUMPER_UP);
    }

    // Capstone release on top of Stone inside robot
    public void holdCapstone() {
        capstoneServo.setPosition(CAP_LOCKED);
    }
    public void releaseCapstone() {
        capstoneServo.setPosition(CAP_RELEASED);
    }

    // set light color methods
    public void setLightGreen() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        lights.setPattern(pattern);
    }
    public void setLightRed() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
        lights.setPattern(pattern);
    }
    public void setSlowBlue() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        lights.setPattern(pattern);
    }
    public void setFastBlue() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        lights.setPattern(pattern);
    }


}


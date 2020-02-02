package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.purepursuit.MathFunctions;

import static org.firstinspires.ftc.teamcode.purepursuit.MathFunctions.angleWrapRad;


public class MecaBotMove {

    enum WheelPosition { LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK }
    public enum DriveType {TANK, MECANUM, DIAGONAL}

    // Encoder based movement calculation constants
    static final int    MOTOR_TICK_COUNT    = 560; // we are using REV HD Hex Planetary 20:1 for drive train
    static final float  mmPerInch           = 25.4f;
    static final double WHEEL_DIA           = 75 / mmPerInch;  // REV mecanum wheels have 75 millimeter diameter
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIA;
    static final double ENCODER_TICKS_PER_INCH      = MOTOR_TICK_COUNT / WHEEL_CIRCUMFERENCE;
    static final int    ENCODER_TICKS_ERR_MARGIN    = 50;
    static final double OUTER_TO_INNER_TURN_SPEED_RATIO = 6.0;

    // Driving speeds
    public static final double DRIVE_SPEED_MIN     = 0.15;
    public static final double DRIVE_SPEED_SLOW    = 0.3;
    public static final double DRIVE_SPEED_DEFAULT = 0.6;
    public static final double DRIVE_SPEED_FAST    = 0.8;
    public static final double DRIVE_SPEED_MAX     = 1.0;
    public static final double ROTATE_SPEED_SLOW   = 0.2;
    public static final double ROTATE_SPEED_DEFAULT= 0.3;
    public static final double ROTATE_SPEED_FAST   = 0.5;
    static final double DEFAULT_SPEED       = 0.6;  //default wheel speed, same as motor power

    // Distance and Time thresholds
    public static final double DIST_MARGIN          = 1.0; // inches
    public static final double DIST_NEAR            = 4.0; // inches
    public static final double DIST_SLOWDOWN        = 16.0; // inches
    public static final double TIMEOUT_DEFAULT      = 5.0; // seconds
    public static final double TIMEOUT_SHORT        = 3.0; // seconds

    // member variables for state
    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private MecaBot             robot;          // Access to the Robot hardware
    private OdometryGlobalPosition globalPosition; // Robot global position tracker
    private Thread              globalPositionThread;
    //private Orientation         angles;         // Robot heading angle obtained from gyro in IMU sensor
    private String              movementStatus          = "";

    // This setting for goBilda 5202 series 26.9:1 motor, encoder counts per rotation = 753.2
    //private int[]               liftStops = {MecaBot.LIFT_BOTTOM, 159, 846, 1533, 2220, 2907, 3594, 4281, 4968, 5655, MecaBot.LIFT_TOP};
    // This setting for goBilda 5202 series 50.9:1 motor, encoder counts per rotation = 1425.2
    private int[]               liftStops = {MecaBot.LIFT_BOTTOM, 300, 1600, 2900, 4200, 5500, 6800, 8100, 9400, 10700, MecaBot.LIFT_TOP};

    /* Constructor */
    public MecaBotMove(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
        globalPosition = robot.initOdometry();
    }

    // Access methods
    public OdometryGlobalPosition getPosition() {
        return globalPosition;
    }

    public void startOdometry() {

        globalPositionThread = new Thread(globalPosition);
        globalPositionThread.start();
    }

    public void stopOdometry() {

        globalPosition.stop();
    }

    public String getMovementStatus() {
        return movementStatus;
    }

    /**
     * Rotate the robot to desired angle position using the built in gyro in IMU sensor onboard the REV expansion hub
     * The angle is specified relative to the start position of the robot when the IMU is initialized
     * and gyro angle is intialized to zero degrees.
     *
     * @param targetAngle   The desired target angle position in degrees
     * @param turnSpeed     The speed at which to drive the motors for the rotation. 0.0 < turnSpeed <= 1.0
     */
/*    public void gyroRotateToHeading(double targetAngle, double turnSpeed) {

        // determine current angle of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotAngle = angles.firstAngle;
        double delta = MathFunctions.angleWrap(targetAngle - robotAngle);
        double prev = delta;
        double direction = (delta >= 0) ? 1.0 : -1.0; // positive angle requires CCW rotation, negative angle requires CW

        while ((delta != 0) && ((delta > 0) == (prev > 0))) { // while the sign of delta and prev is same (both +ve or both -ve)

            if ((Math.abs(delta) < 10) && (turnSpeed >= ROTATE_SPEED_DEFAULT)) { // slow down when less than 10 degrees rotation remaining
                turnSpeed = DRIVE_SPEED_MIN;
            }
            // the sign of turnSpeed determines the direction of rotation of robot
            robot.driveTank(0, turnSpeed * direction);

            angles = robot.imu.getAngularOrientation();
            robotAngle = angles.firstAngle;
            prev = delta;
            delta = MathFunctions.angleWrap(targetAngle - robotAngle);

            myOpMode.telemetry.addLine("Gyro Angle ")
                    .addData("target", "%.2f", targetAngle)
                    .addData("robot", "%.2f", robotAngle)
                    .addData("delta", "%.2f", delta);
            myOpMode.telemetry.update();
        }
        robot.stopDriving();
    }

    public void gyroRotateToHeading(double targetAngle) {
        gyroRotateToHeading(targetAngle, ROTATE_SPEED_DEFAULT);
    }
*/
    /**
     * Return the Z-axis angle reading from the Gyro on the IMU sensor inside expansion hub
     * This method will return a valid value only if one of the above gyroXYZ() methods have been called
     * resulting in the IMU sensor being read recently.
     * Otherwise the last saved value will be returned, which does not match actual robot heading
     *
     * @return The angle reading in degrees. Positive value is counter clockwise from initial zero position
     * Negative value is clock wise from initial zero. Angle value wraps around at 180 and -180 degrees.
     */
/*    public double getGyroHeading() {
        if (angles == null) {
            return NaN;
        }
        return angles.firstAngle;
    }
*/
    /**
     * Rotate the robot to desired angle position using the odometry position feedback
     *
     * @param targetAngle   The desired target angle position in degrees
     * @param turnSpeed     The speed at which to drive the motors for the rotation. 0.0 < turnSpeed <= 1.0
     */
    public void odometryRotateToHeading(double targetAngle, double turnSpeed, double timeout, boolean slowDownAtEnd) {

        ElapsedTime runtime = new ElapsedTime();

        // determine current angle of the robot
        double robotAngle = globalPosition.getOrientationDegrees();
        double delta = MathFunctions.angleWrap(targetAngle - robotAngle);
        double prev = delta;
        double direction = (delta >= 0) ? 1.0 : -1.0; // positive angle requires CCW rotation, negative angle requires CW

        movementStatus = String.format("Rotate To Angle =%.1f, Spd=%1.1f TO=%1.1f", targetAngle, turnSpeed, timeout);
        runtime.reset();
        // while the sign of delta and prev is same (both +ve or both -ve) run loop
        while (myOpMode.opModeIsActive() && runtime.seconds() < timeout && ((delta > 0) == (prev > 0))) {

            if (slowDownAtEnd && (Math.abs(delta) < 5)) { // slow down when last few degrees rotation remaining
                turnSpeed = DRIVE_SPEED_MIN;
            }
            // the sign of turnSpeed determines the direction of rotation of robot
            robot.driveTank(0, turnSpeed * direction);

            robotAngle = globalPosition.getOrientationDegrees();
            prev = delta;
            delta = MathFunctions.angleWrap(targetAngle - robotAngle);

            myOpMode.telemetry.addData("Rotate", "Robot=%.2f, Rel Angle=%.2f", robotAngle, delta);
            myOpMode.telemetry.update();
        }
        robot.stopDriving();
        movementStatus = String.format("Done To Angle=%.1f, Spd=%1.1f in T=%1.1f", targetAngle, turnSpeed, runtime.seconds());
    }

    public void odometryRotateToHeading(double targetAngle) {
        odometryRotateToHeading(targetAngle, ROTATE_SPEED_DEFAULT, TIMEOUT_SHORT, true);
    }

    /**
     * Drive the robot at specified speed towards the specified target position on the field.
     * The current position of the robot obtained using odometry readings.
     *
     * @param x     Target position global x coordinate value (inches)
     * @param y     Target position global y coordinate value (inches)
     * @param speed Speed/Power used to drive. Must be in range of -1.0 <= speed <= 1.0a
     * @param timeout Time (seconds) to complete the move or abort
     * @param slowDownAtEnd true if robot should slow down close to destination, to avoid overshooting
     */
    public void goToPosition(double x, double y, double speed, double timeout, boolean slowDownAtEnd) {

        ElapsedTime runtime = new ElapsedTime();
        double distance = 200; // greater than the diagonal length of the FTC field
        double previous;

        movementStatus = String.format("GoToPos X=%3.2f, Y=%2.2f, Spd=%1.1f, TO=%1.1f", x, y, speed, timeout);
        speed = Range.clip(speed, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
        runtime.reset();
        // we consider reaching the destination (x,y) position if less than DIST_MARGIN inches away OR
        // if the distance from the destination starts increases from its previous value
        while (myOpMode.opModeIsActive() && (runtime.seconds() < timeout) && (distance >= DIST_MARGIN)) {
            previous = distance;
            distance = goTowardsPosition(x, y, speed, slowDownAtEnd);
            // Avoid oscillations at the end, near the target destination (DIST_NEAR threshold)
            // If the distance from the destination starts increasing from its previous value,
            // then robot may have overshot the target coordinate location, stop there
            if  (distance < DIST_NEAR && distance > previous) {
                break;
            }
        }
        robot.stopDriving();
        movementStatus = String.format("Reached X=%3.2f, Y=%2.2f, Spd=%1.1f in T=%1.1f", x, y, speed, runtime.seconds());
    }

    public void goToPosition(double x, double y, double speed, double timeout) {

        goToPosition(x, y, speed, timeout, true);
    }

    public void goToPosition(double x, double y) {

        goToPosition(x, y, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Drive the robot at specified speed towards the specified target position on the field.
     * Note that we are not waiting to reach the target position. This method only sets wheel power
     * in the direction of the target position and must be called again repeatedly at an update
     * interval, typically 50ms - 75ms.
     * The current position of the robot obtained using odometry readings and wheel power are both
     * calculated again at each call to this method.
     *
     * @param x     Target position global x coordinate value (inches)
     * @param y     Target position global y coordinate value (inches)
     * @param speed Speed/Power used to drive. Must be in range of -1.0 <= speed <= 1.0
     * @return <em>true</em> if sucessfully issued command to robot to drive, <em>false</em> if reached the destination
     */
    public double goTowardsPosition(double x, double y, double speed, boolean slowDownAtEnd) {

        double distanceToPosition = Math.hypot(globalPosition.getXinches() - x,  globalPosition.getYinches() - y);
        double absoluteAngleToPosition = Math.atan2(y - globalPosition.getYinches(), x - globalPosition.getXinches());
        double relativeAngleToPosition;

        if (robot.isFrontIntake()) {
            relativeAngleToPosition = angleWrapRad(absoluteAngleToPosition - globalPosition.getOrientationRadians());
        }
        else { // (robot.isFrontLiftarm())
        // override in case of Robot front face has been REVERSED. The motors will run swapped (left front runs as right back)
        // The only control we need to change is to calculate the turn power for driving in reverse direction
        // This is done by adding 180 degrees (or PI radians) to the relative Angle (or to the robot orientation angle)
            relativeAngleToPosition = angleWrapRad(absoluteAngleToPosition - globalPosition.getOrientationRadians() + Math.PI);
        }

        double drivePower = speed;
        // when within DIST_SLOWDOWN inches of target, reduce the speed proportional to remaining distance to target
        if (slowDownAtEnd && distanceToPosition < DIST_SLOWDOWN) {
            // however absolute minimum power is required otherwise the robot cannot move the last couple of inches

            // enable this code for delayed aggressive braking - overshoots target but saves some time
            //double slowPower = Range.clip(distanceToPosition / DIST_SLOWDOWN, DRIVE_SPEED_MIN, DRIVE_SPEED_MAX);
            //drivePower = Math.min(speed, slowPower);

            // enable this code instead for more gentle slow down at end - stop closer to target position
            double slowPower = (distanceToPosition / DIST_SLOWDOWN) * speed;
            drivePower = Math.max(DRIVE_SPEED_MIN, slowPower);
        }
        // set turnspeed proportional to the amount of turn required, however beyond 30 degrees turn, full speed is ok
        // note here that positive angle means turn left (since angle is measured counter clockwise from X-axis)
        // this must match the behavior of MecaBot.DriveTank() method used below.
        double turnPower = Range.clip(relativeAngleToPosition / Math.toRadians(30), -1.0, 1.0) * drivePower;
        // no minimum for turnPower until robot auto driving tests indicate a need.

        myOpMode.telemetry.addLine("GoToPos ").addData("Dist", " %4.2f in", distanceToPosition).addData("Rel Angle", " %4.2f deg", Math.toDegrees(relativeAngleToPosition));
        myOpMode.telemetry.addLine("Power ").addData("drive", " %.2f", drivePower).addData("turn", "%.2f", turnPower);
        myOpMode.telemetry.update();

        // let's stop driving when within a short distance of the destination. This threshold may need to be tuned.
        // A threshold is necessary to avoid oscillations caused by overshooting of target position.
        // This check could be done early in the method, however it is done towards end deliberately to get telemetry readouts
        if (distanceToPosition < DIST_MARGIN) {
            robot.stopDriving();
        }
        else {
            robot.driveTank(drivePower, turnPower);
        }

        return distanceToPosition;
    }

    /**
     * Move to a position at specified X-coordinate, while maintaining the current Y-coordinate value
     *
     * @param targetX   the target X coordinate position
     * @param speed     driving speed for movement
     */
    public void goToXPosition(double targetX, double speed, double timeout) {

        double curX = globalPosition.getXinches();

        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(targetX - curX) < DIST_MARGIN) {
            return;
        }

        // THIS CODE IS INCOMPATIBLE WITH DRIVING THE ROBOT IN REVERSE DIRECTION
//        // First lets point heading in the direction of movement, so we can drive straight
//        if (targetX - curX > 0) { // if we need to move towards positive X-Axis
//            gyroRotateToHeading(0.0, ROTATE_SPEED_SLOW);
//        }
//        else { // we need to move towareds negative X-Axis
//            gyroRotateToHeading(180.0, ROTATE_SPEED_SLOW);
//        }

        // the gyro rotation moves the robot X,Y position, we will ignore that small amount
        // get the Y coorindate now (after gyroRotate()) so we drive stright only along X-axis
        double curY = globalPosition.getYinches();

        // Now lets go to the target destination coordinate
        goToPosition(targetX, curY, speed, timeout);
    }

    public void goToXPosition(double targetX) {
        goToXPosition(targetX, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move to a position at specified Y-coordinate, while maintaining the current X-coordinate value
     *
     * @param targetY   the target Y coordinate position
     * @param speed     driving speed for movement
     */
    public void goToYPosition(double targetY, double speed, double timeout) {

        double curY = globalPosition.getYinches();

        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(targetY - curY) < DIST_MARGIN) {
            return;
        }

        // THIS CODE IS INCOMPATIBLE WITH DRIVING THE ROBOT IN REVERSE DIRECTION
//        // First lets point heading in the direction of movement, so we can drive straight
//        if (targetY - curY > 0) { // if we need to move towards positive Y-Axis
//            gyroRotateToHeading(90.0, ROTATE_SPEED_SLOW);
//        }
//        else { // we need to move towareds negative Y-Axis (which is off the field, so never reach here)
//            gyroRotateToHeading(-90.0, ROTATE_SPEED_SLOW);
//        }

        // the gyro rotation moves the robot X,Y position, we will ignore that small amount
        // get the X coorindate now (after gyroRotate()) so we drive stright only along X-axis
        double curX = globalPosition.getXinches();

        // Now lets go to the target destination coordinate
        goToPosition(curX, targetY, speed, timeout);
    }

    public void goToYPosition(double targetY) {
        goToYPosition(targetY, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses odometry feedback to determine Robot current position during movement
     * Move FORWARD : inches +ve value, mecanumSideways = false;
     * Move REVERSE : inches -ve value, mecanumSideways = false;
     * Move RIGHT   : inches +ve value, mecanumSideways = true;
     * Move LEFT    : inches -ve value, mecanumSideways = true;
     * @param inches            distance to move
     * @param driveType         driving method (tank, mecanum, and diagonal)
     * @param speed             driving speed for the movement
     * @param timeout           Time (seconds) to complete the move or abort
     */
    public void odometryMoveDistance(double inches, DriveType driveType, double speed, double timeout) {
        // do not move less than 1 inch, that is our margin threshold for reaching the target coordinate.
        if (Math.abs(inches) < DIST_MARGIN) {
            return;
        }
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if distance is negative, direction must be reversed, both forwards/backwards or left/right side
        // which is done by negative power to the motors
        if (inches < 0) {
            speed = -speed;
        }
        double origX = globalPosition.getXinches();
        double origY = globalPosition.getYinches();
        double curX;
        double curY;
        double distance = 0;
        ElapsedTime runtime = new ElapsedTime();

        movementStatus = String.format("Dist %.1f in, from (%.1f, %.1f) S=%1.1f TO=%1.1f", inches, origX, origY, speed, timeout);

        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < timeout) && (distance < Math.abs(inches))) {
            switch (driveType) {
                case MECANUM:
                    robot.driveMecanum(speed);
                    break;
                case DIAGONAL:
                    robot.driveDiagonal(speed);
                    break;
                case TANK:
                    robot.driveTank(speed, 0.0);
                    break;
            }
            curX = globalPosition.getXinches();
            curY = globalPosition.getYinches();
            distance = Math.hypot((curX - origX), (curY - origY));

            myOpMode.telemetry.addLine("MoveDist ").addData("now at", "%.1f of %.1f in", distance, inches);
            myOpMode.telemetry.update();
        }
        // Reached within threshold of target distance.
        robot.stopDriving();
        movementStatus = String.format("Done D=%.1f in, from (%.1f, %.1f) S=%1.1f in T=%1.1f", distance, origX, origY, speed, runtime.seconds());

    }

    public void odometryMoveDistance(double inches, DriveType driveType, double speed) {
        odometryMoveDistance(inches, driveType, speed, TIMEOUT_DEFAULT);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses odometry feedback to determine Robot current position during movement
     * Move FORWARD : inches +ve value, mecanumSideways = false;
     * Move REVERSE : inches -ve value, mecanumSideways = false;
     * Move RIGHT   : inches +ve value, mecanumSideways = true;
     * Move LEFT    : inches -ve value, mecanumSideways = true;
     * @param inches            distance to move
     * @param driveType         driving method (tank, mecanum, and diagonal)
     */
    public void odometryMoveDistance(double inches, DriveType driveType) {
        odometryMoveDistance(inches, driveType, DRIVE_SPEED_DEFAULT, TIMEOUT_DEFAULT);
    }

    /**
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     * @param inches distance to move
     */
    public void odometryMoveForwardBack(double inches) {
        odometryMoveDistance(inches, DriveType.TANK, DRIVE_SPEED_DEFAULT);
    }

    /**
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     * @param inches distance to move
     * @param speed  speed of movement
     */
    public void odometryMoveForwardBack(double inches, double speed) {
        odometryMoveDistance(inches, DriveType.TANK, speed, TIMEOUT_DEFAULT);
    }

    /**
     * Move robot left or right, +ve distance moves right, -ve distance moves left
     * @param inches distance to move
     */
    public void odometryMoveRightLeft(double inches) {
        odometryMoveDistance(inches, DriveType.MECANUM);
    }

    /**
     * Move robot left or right, +ve distance moves right, -ve distance moves left
     * @param inches distance to move
     * @param speed  speed of movement
     */
    public void odometryMoveRightLeft(double inches, double speed) {
        odometryMoveDistance(inches, DriveType.MECANUM, speed, TIMEOUT_DEFAULT);
    }

    /*
     * Move robot forward or backward, +ve distance moves forward, -ve distance moves backward
     */
    public void encoderMoveForwardBack(double inches) {
        encoderMoveDistance( inches, false, DEFAULT_SPEED);
    }

    public void encoderMoveForwardBack(double inches, double speed) {
        encoderMoveDistance( inches, false, speed);
    }

   /*
    * Move robot left or right, +ve distance moves right, -ve distance moves left
    */
    public void encoderMoveRightLeft(double inches) {
        encoderMoveDistance(inches, true, DEFAULT_SPEED);
    }

    public void encoderMoveRightLeft(double inches, double speed) {
        encoderMoveDistance(inches, true, speed);
    }

    /*
     * Move robot left or right, +ve distance moves LEFT, -ve distance moves RIGHT
     */
    @Deprecated
    public void encoderMoveLeftRight(double inches) {
        encoderMoveDistance(inches * -1.0, true, DEFAULT_SPEED);
    }

    @Deprecated
    public void encoderMoveLeftRight(double inches, double speed) {
        encoderMoveDistance(inches * -1.0, true, speed);
    }

    /**
     * Move the specified distance (in inches), either normal or mecanum sideways movement.
     * The movement direction is controlled by the sign of the first parameter, distance in inches to move
     * This method uses the wheel encoders to move exactly the desired distance.
     * @param inches            distance to move
     * @param mecanumSideways   Mecanum sideways movement if true, Normal tank movement if false
     * @param speed             driving speed for the movement
     */

    private void encoderMoveDistance(double inches, boolean mecanumSideways, double speed) {

        //convert inches to tick counts
        int driverEncoderTarget = (int) (ENCODER_TICKS_PER_INCH * inches);

        // default is drive straight all wheels drive same direction (forward or backward depending on sign)
        int leftFront = driverEncoderTarget;
        int leftBack = driverEncoderTarget;
        int rightFront = driverEncoderTarget;
        int rightBack = driverEncoderTarget;

        // for mecanum sideways movement, move Right when goForwardOrRight is true
        // Right wheels move inside, Left wheels move outside
        if (mecanumSideways) {
            rightFront = -driverEncoderTarget;  // drive backward for inside
            rightBack = driverEncoderTarget;    // drive forward for inside
            leftFront = driverEncoderTarget;    // drive forward for outside
            leftBack = -driverEncoderTarget;    // drive backward for outside
        }
        // same code above works for mecanum move Left also. False value of goForwardOrRight already flipped the sign above

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of the motors to whatever speed is needed
        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(WheelPosition.RIGHT_FRONT, leftFront, leftBack, rightFront, rightBack);

    }

    // Rotate around Robot own center
    public void encoderRotate(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot,
        // counterClockwise means Right wheels turning forward, Left wheels turning backwards

        int ticks = (int) (ENCODER_TICKS_PER_INCH * inches);

        int leftFront = counterClockwise ? -ticks : +ticks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? +ticks : -ticks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(speed);

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", inches).addData("inner", inches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderRotate(double inches, boolean counterClockwise) {
        encoderRotate(inches, counterClockwise, DEFAULT_SPEED);
    }

    // Turn in an arc
    // Assume robot is touching the outer edge of a tile and we want to rotate it in a circular arc
    // This is not rotating around its center axis but an arc with center of circle outside the robot where the tiles meet
    // Outer wheels run along outer circle and inner wheels run along an inner circle

    public void encoderTurn(double inches, boolean counterClockwise, double speed) {

        // Green intake wheels is front of robot, counterClockwise means Right wheels on outer circle
        double outerWheelInches = inches;
        double innerWheelInches = inches / OUTER_TO_INNER_TURN_SPEED_RATIO;

        int outerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * outerWheelInches);
        int innerWheelTicks = (int) (ENCODER_TICKS_PER_INCH * innerWheelInches);

        int leftFront = counterClockwise ? innerWheelTicks : outerWheelTicks;
        int leftBack = leftFront;
        int rightFront = counterClockwise ? outerWheelTicks : innerWheelTicks;
        int rightBack = rightFront;

        // set target position for encoder Drive
        robot.resetDriveEncoder();
        robot.setTargetPosition(leftFront, leftBack, rightFront, rightBack);

        // Set the motors to run to the necessary target position
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double wheelPower = Range.clip(speed, 0.0, 1.0);
        double insideWheelPower = wheelPower / OUTER_TO_INNER_TURN_SPEED_RATIO;
        if (counterClockwise) {
            robot.setDrivePower(insideWheelPower, wheelPower);
        } else {
            robot.setDrivePower(wheelPower, insideWheelPower);
        }

        myOpMode.telemetry.addLine("Driving inches | ").addData("outer", outerWheelInches).addData("inner", innerWheelInches);

        // loop until motors are busy driving, update current position on driver station using telemetry
        waitToReachTargetPosition(counterClockwise ? WheelPosition.RIGHT_FRONT : WheelPosition.LEFT_FRONT,
                leftFront, leftBack, rightFront, rightBack);
    }

    public void encoderTurn(double inches, boolean counterClockwise) {
        encoderTurn(inches, counterClockwise, DEFAULT_SPEED);
    }

    private void waitToReachTargetPosition(WheelPosition dominantWheel, int leftFront, int leftBack, int rightFront, int rightBack) {

        // Loop until motors are no longer busy.
        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {

            myOpMode.telemetry.addLine("Target position | ").addData("LF", leftFront).addData("RF", rightFront);
            myOpMode.telemetry.addLine("Target position | ").addData("LB", leftBack).addData("RB", rightBack);
            myOpMode.telemetry.addLine("Current position | ").addData("LF", robot.leftFrontDrive.getCurrentPosition()).addData("RF", robot.rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addLine("Current position | ").addData("LB", robot.leftBackDrive.getCurrentPosition()).addData("RB", robot.rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();

            //encoder reading a bit off target can keep us in this loop forever, so given an error margin here
            if ((dominantWheel == WheelPosition.LEFT_FRONT) && Math.abs(robot.leftFrontDrive.getCurrentPosition() - leftFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.LEFT_BACK) && Math.abs(robot.leftBackDrive.getCurrentPosition() - leftBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_FRONT) && Math.abs(robot.rightFrontDrive.getCurrentPosition() - rightFront) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
            else if ((dominantWheel == WheelPosition.RIGHT_BACK) && Math.abs(robot.rightBackDrive.getCurrentPosition() - rightBack) < ENCODER_TICKS_ERR_MARGIN) {
                break;
            }
        }

        // Stop powering the motors - robot has moved to intended position
        robot.stopDriving();
        myOpMode.telemetry.addData("DONE driving LF position=",robot.leftFrontDrive.getCurrentPosition());
        // Turn off RUN_TO_POSITION
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.sleep(250);

    }

    public void moveLiftToPosition(int position) {

        robot.moveLift(position);
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && robot.liftMotor.isBusy() && runTime.seconds() < 1.5) {
            myOpMode.telemetry.addData("LIFT ", "auto moving to %d", position);
            myOpMode.telemetry.update();
        }
        robot.stopLift();
    }

    public void moveLiftDown() {
        int pos = robot.liftMotor.getCurrentPosition();
        int target = pos;
        for (int i=0; i<liftStops.length - 1; i++) {
            // whatever lift stop is higher or equal to current position, go down one stop below
            if (liftStops[i+1] - pos > -100) {
                target = liftStops[i];
                break;
            }
        }
        this.moveLiftToPosition(target);
    }

    public void moveLiftUp() {
        int pos = robot.liftMotor.getCurrentPosition();
        int target = pos;
        // whatever lift stop is higher than current position, go up to that stop
        for (int i=0; i<liftStops.length; i++) {
            if (liftStops[i] - pos > 100) {
                target = liftStops[i];
                break;
            }
        }
        this.moveLiftToPosition(target);
    }

    public void moveLiftArmToPosition(int position) {

        robot.moveLiftArm(position);
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && robot.liftArmMotor.isBusy() && runTime.seconds() < 1.5) {
            myOpMode.telemetry.addData("ARM ", "auto moving to %d", position);
            myOpMode.telemetry.update();
        }
        robot.stopLiftArm();
    }

    public void moveLiftArmInside() {
        this.moveLiftArmToPosition(MecaBot.ARM_INSIDE);
    }

    public void moveLiftArmOutside() {
        this.moveLiftArmToPosition(MecaBot.ARM_OUTSIDE);
    }


    /*
 * The code below is obsolete after the 24-NOV-2019 Qualifying Tournament.
 * 1. The Robot does not have a side arm, the robot heading will always towards the stone side for both RED and BLUE
 * 2. Since we are using odometry we cannot flip the X-axis positive direction between RED and BLUE.
 */

    public static final boolean BLUESIDE =false;      //if red side, set it to false
    public static double robotStartX= 41;      // robot origin aline with image right
    public static double robotStartY=17.25;       //right back corner of robot

    private static final boolean START_STONE_SIDE=true;  //true if start at stone side
    private static final double X_PARK_INNER_OUTER  = 9.0;

    public static void initRobotStartX(){
        if (START_STONE_SIDE) {
            if (BLUESIDE) {
                robotStartX = 41.5;   //blue stone side: align with wall image right edge
            } else {
                robotStartX = 24;   //red stone side: align with tile edge
            }
        }else{     //foundation side
            if (BLUESIDE) {
                robotStartX = -24;   //blue foundation side: align tile
            } else {
                robotStartX = -41;   //red foundation side: align with tile edge
            }
        }
    }


    /**
     *  Method to drive the robot from one point marked by coordinate curX, curY to another targetX, target Y.
     *  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
     *  with Y pointing to the middle of the field and X pointing to the stone side.
     *  Robot move in Y direction first followed by X.
     *    @param targetX  target X coordinate value
     *    @param targetY  target Y coordinate value
     *    @param curX     start X coordinate value
     *    @param curY     start Y coordinate value
     *    @param headXpositive   true when robot forward heading is along positive X axis
     *                           set this to true for RED alliance, false for BLUE alliance
     */
    public void moveYX(double targetX, double targetY, double curX, double curY, boolean headXpositive){
        double distanceMarginInch=0.1;    //minimum required distance to invoke the move
        double xdist=targetX-curX;
        double ydist=targetY-curY;
        if (!headXpositive){
            //blue side: negative x require robot to go forward
            xdist=-xdist;
        }
        ydist=-ydist;  //both side: positive y movement require robot to go right

        myOpMode.telemetry.addData(" Xdist Ydist", "%.1f %.1f", xdist,ydist);
        if (Math.abs(ydist) > distanceMarginInch){
            encoderMoveLeftRight(ydist);
        }
        if (Math.abs(xdist) > distanceMarginInch) {
            encoderMoveForwardBack(xdist);
        }
    }

    //value adjust for RED side stone pick up and return to parking
    private static final double parkingMarginL=1;   //leave space on left side of robot at parking
    private static final double parkingMarginR=0;   //leave space on right/arm side of robot at parking
    private static final double bridgeY=46;

    /**
     *  Method to park the robot under the bridge. It used the method moveYX() method
     *  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
     *  with Y pointing to the middle of the field and X pointing to the stone side.
     *    @param curX     start X coordinate value
     *    @param curY     start Y coordinate value
     *    @param parkInside      park under the bridge on inside tile towards center of field (or outside tile towards perimeter)
     *    @param headXpositive   true when robot forward heading is along positive X axis
     *                           set this to true for RED alliance, false for BLUE alliance
     */
    public void goPark(double curX, double curY, boolean parkInside, boolean headXpositive){

        //adjusted temporarily for Red Alliance Parking
        double toY = parkInside ? bridgeY- parkingMarginR:
                 MecaBot.WIDTH+ parkingMarginL;
        double toX = headXpositive? -5:X_PARK_INNER_OUTER;

        myOpMode.telemetry.addData("Parking target X Y", "%.1f %.1f", toX,toY);
        moveYX(toX,toY,curX,curY,headXpositive);
    }

}
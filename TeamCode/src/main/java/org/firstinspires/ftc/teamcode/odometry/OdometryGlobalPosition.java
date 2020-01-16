package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.purepursuit.MathFunctions;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Vishesh Goyal on 12/24/2019
 *
 * All the position values are initialized to zero, including the robot orientation angle theta.
 * The implemention by Sarthak computed the change in robot angle using (left encoder count - right encoder count) indicating clockwise turn is positive angle.
 * Also the vector projection formula used for calculating global position indicates robot angle is measured from Y-Axis
 *
 * Modified by Vishesh to follow the conventional polar coordinate system, so we use this code with other code modules such as pure pursuit
 * change in robot angle = (right encoder count - left encoder count) indicating counter-clockwise rotation is positive angle value
 * Vector projection formula used for calculating global position, now uses robot angle (robotAngleRad) measured CCW from X-Axis
 *
 */
public class OdometryGlobalPosition implements Runnable{

    //Odometry wheels
    private DcMotor verticalLeftEncoder, verticalRightEncoder, horizontalEncoder;

    //Position variables used for storage and calculations
    private double robotGlobalX = 0, robotGlobalY = 0, robotAngleRad = Math.PI/2; // robot starts with 90 degree angle in direction of positive Y-axis
    private double verticalRightCount = 0, verticalLeftCount = 0, horizontalCount = 0;
    private double prevVerticalRightCount = 0, prevVerticalLeftCount = 0, prevHorizontalCount = 0;

    private int verticalLeftEncoderDirection = 1;
    private int verticalRightEncoderDirection = 1;
    private int horizontalEncoderDirection = 1;

    //Algorithm constants
    private final double WHEELBASE_SEPARATION_COUNT;
    private final double HORIZONTAL_COUNT_PER_RADIAN;
    private final double ENCODER_COUNT_PER_INCH;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalCountPerRadianFile = AppUtil.getInstance().getSettingsFile("horizontalCountPerRadian.txt");

    //Thread run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalLeftEncoder left odometry encoder, facing the vertical direction
     * @param verticalRightEncoder right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalPosition(DcMotor verticalLeftEncoder, DcMotor verticalRightEncoder, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay){
        this.verticalLeftEncoder = verticalLeftEncoder;
        this.verticalRightEncoder = verticalRightEncoder;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        this.WHEELBASE_SEPARATION_COUNT = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.HORIZONTAL_COUNT_PER_RADIAN = Double.parseDouble(ReadWriteFile.readFile(horizontalCountPerRadianFile).trim());
        this.ENCODER_COUNT_PER_INCH = COUNTS_PER_INCH;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalPositionUpdate(){
        //Get Current Positions
        verticalLeftCount = (verticalLeftEncoder.getCurrentPosition() * verticalLeftEncoderDirection);
        verticalRightCount = (verticalRightEncoder.getCurrentPosition() * verticalRightEncoderDirection);
        horizontalCount = (horizontalEncoder.getCurrentPosition()* horizontalEncoderDirection);

        double leftChange = verticalLeftCount - prevVerticalLeftCount;
        double rightChange = verticalRightCount - prevVerticalRightCount;

        //Calculate Angle
        // These formulas assume that robotAngleRad is positive when Robot is turning counter-clockwise
        // All local variables are a signed value, representing change or angle direction
        double changeInRobotAngle = (rightChange - leftChange) / (WHEELBASE_SEPARATION_COUNT);
        robotAngleRad = MathFunctions.angleWrapRad(robotAngleRad + changeInRobotAngle);

        //Get the components of the motion
        double rawHorizontalChange = horizontalCount - prevHorizontalCount;
        double horizontalChange = rawHorizontalChange + (changeInRobotAngle * HORIZONTAL_COUNT_PER_RADIAN);

        // p is the vector of forward movement of the Robot, direction parallel to drivetrain wheels
        // n is the vector of normal movement of thee Robot, direction perpendicular to drivetrain wheels
        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        // Calculate and update the position using Vector projection formula
        // These formulas assume robotAngleRad is measured from X-Axis turning counter-clockwise, same as angle theta in a polar coordinate system
        robotGlobalX = robotGlobalX + (p*Math.cos(robotAngleRad) + n*Math.sin(robotAngleRad));
        robotGlobalY = robotGlobalY + (p*Math.sin(robotAngleRad) - n*Math.cos(robotAngleRad));

        prevVerticalLeftCount = verticalLeftCount;
        prevVerticalRightCount = verticalRightCount;
        prevHorizontalCount = horizontalCount;
    }

    /**
     * Sets or intializes the x,y coordinates and orientation angle theta of the robot global position on the FTC field
     * There is no error checking, if illegal values are passed as parameters, then subsequent behavior is undefined.
     * The field x and y coordinates can have max value of +- 70.375 inches from center of the field
     * (Calculation: Each tile is 23.5 inches square and 0.75 inch tile tabs are cut at the field edges,
     *  total 6 tiles = 140.25 inches width, add 0.25 margin from tile to wall on each side. Total 140.75 inches wall to wall)
     * The Robot x and y coordinates are at the center of the robot. For robot width of 18 inches, the robot X,Y will be 9 inches away from field wall.
     *
     * @param x X-coordinate value in inches, -61 <= x <= 61
     * @param y Y-coordinate value in inches, -61 <= y <= 61
     * @param deg orientation angle in Degrees, -180 < deg <= 180, measure from X-Axis, CCW is positive and CW is negative
     */
    public void initGlobalPosition(double x, double y, double deg) {
        robotGlobalX = x * ENCODER_COUNT_PER_INCH;
        robotGlobalY = y * ENCODER_COUNT_PER_INCH;
        robotAngleRad = Math.toRadians(deg);
    }
    /**
     * Returns the robot's global orientation in Radians unit
     * @return global orientation angle
     */
    public double getOrientationRadians() {
        return robotAngleRad;
    }

    /**
     * Returns the robot's global orientation in Degrees unit
     * @return global orientation angle
     */
    public double getOrientationDegrees() {
        return Math.toDegrees(robotAngleRad);
    }

    /**
     * Returns the robot's global x coordinate on the field in inches
     * @return global x coordinate
     */
    public double getXinches() {
        return robotGlobalX / ENCODER_COUNT_PER_INCH;
    }

    /**
     * Returns the robot's global y coordinate on the field in inches
     * @return global y coordinate
     */
    public double getYinches() {
        return robotGlobalY / ENCODER_COUNT_PER_INCH;
    }

    /**
     * Returns the robot's global x coordinate in encoder count
     * @return global x coordinate
     */
    public double getXCount() {
        return robotGlobalX;
    }

    /**
     * Returns the robot's global y coordinate in encoder count
     * @return global y coordinate
     */
    public double getYCount() {
        return robotGlobalY;
    }

    /**
     * Returns the vertical left encoder's tick count
     * @return Vertical Left Encoder count
     */
    public double getVerticalLeftCount() {
        return verticalLeftCount;
    }

    /**
     * Returns the vertical right encoder's tick count
     * @return Vertical Right Encoder count
     */
    public double getVerticalRightCount() {
        return verticalRightCount;
    }

    /**
     * Returns the horizontal encoder's tick count
     * @return Horizontal Encoder count
     */
    public double getHorizontalCount() {
        return horizontalCount;
    }

    /**
     * Stops the position update thread
     */
    public void stop() {
        isRunning = false;
    }

    public void reverseLeftEncoder(){
        verticalLeftEncoderDirection *= -1;
    }

    public void reverseRightEncoder(){
        verticalRightEncoderDirection *= -1;
    }

    public void reverseNormalEncoder(){
        horizontalEncoderDirection *= -1;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

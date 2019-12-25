package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 *
 * All the position values are initialized to zero, including the robot orientation angle theta
 * Further the change in robot angle is computed (see below) using (left encoder count - right encoder count) indicating clockwise turn is positive angle.
 * The vector projection formula used for calculating global position indicates robot angle is measured from Y-Axis
 *
 */
public class OdometryGlobalPosition implements Runnable{
    //Odometry wheels
    private DcMotor verticalLeftEncoder, verticalRightEncoder, horizontalEncoder;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    private double robotGlobalX = 0, robotGlobalY = 0, robotAngleRad = 0;
    private double verticalRightCount = 0, verticalLeftCount = 0, horizontalCount = 0;
    private double prevVerticalRightCount = 0, prevVerticalLeftCount = 0, prevHorizontalCount = 0;

    //Algorithm constants
    private double wheelBaseSeparationCount;
    private double horizontalCountPerRadian;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalCountPerRadianFile = AppUtil.getInstance().getSettingsFile("horizontalCountPerRadian.txt");

    private int verticalLeftEncoderDirection = 1;
    private int verticalRightEncoderDirection = 1;
    private int horizontalEncoderDirection = 1;

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

        this.wheelBaseSeparationCount = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalCountPerRadian = Double.parseDouble(ReadWriteFile.readFile(horizontalCountPerRadianFile).trim());

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
        // These formulas assume that robotAngleRad is positive turning clockwise
        double changeInRobotAngle = (leftChange - rightChange) / (wheelBaseSeparationCount);
        robotAngleRad = ((robotAngleRad + changeInRobotAngle));

        //Get the components of the motion
        double rawHorizontalChange = horizontalCount - prevHorizontalCount;
        double horizontalChange = rawHorizontalChange - (changeInRobotAngle * horizontalCountPerRadian);

        // p is the vector of forward movement of the Robot, direction parallel to drivetrain wheels
        // n is the vector of normal movement of thee Robot, direction perpendicular to drivetrain wheels
        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        // Calculate and update the position
        // These formulas assume robotAngleRad is measured from Y-Axis turning clockwise
        // This seems an unconventional choice, typically angle theta is counter-clockwise turn from X-Axis
        robotGlobalX = robotGlobalX + (p*Math.sin(robotAngleRad) + n*Math.cos(robotAngleRad));
        robotGlobalY = robotGlobalY + (p*Math.cos(robotAngleRad) - n*Math.sin(robotAngleRad));

        prevVerticalLeftCount = verticalLeftCount;
        prevVerticalRightCount = verticalRightCount;
        prevHorizontalCount = horizontalCount;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double getXCount() {
        return robotGlobalX;
    }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double getYCount() {
        return robotGlobalY;
    }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double GetOrientationDegrees() {
        return Math.toDegrees(robotAngleRad) % 360;
    }

    /**
     * Returns the vertical right encoder's tick count
     * @return Vertical Right Encoder count
     */
    public double getVerticalRightCount() {
        return verticalRightCount;
    }

    /**
     * Returns the vertical left encoder's tick count
     * @return Vertical Left Encoder count
     */
    public double getVerticalLeftCount() {
        return verticalLeftCount;
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
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderDirection == 1){
            verticalLeftEncoderDirection = -1;
        }else{
            verticalLeftEncoderDirection = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderDirection == 1){
            verticalRightEncoderDirection = -1;
        }else{
            verticalRightEncoderDirection = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(horizontalEncoderDirection == 1){
            horizontalEncoderDirection = -1;
        }else{
            horizontalEncoderDirection = 1;
        }
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

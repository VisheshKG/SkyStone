package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Vishesh Goyal on 12/24/2019
 * - adaptation to Team13345 MecaBot hardware
 * - Detailed comments to explain the calculation of wheel base separation. Improved variable names for better understanding.
 * - Bug fixes in code and comments
 *
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //IMU Sensor
    BNO055IMU imu;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
    String rfName = "rightFrontDrive", rbName = "rightBackDrive", lfName = "leftFrontDrive", lbName = "leftBackDrive";
    String verticalLeftEncoderName = "leftIntake", verticalRightEncoderName = "rightIntake", horizontalEncoderName = "horizontalEncoder";

    final double PIVOT_SPEED = 0.4;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 242.552133272048492;  // FTC Team 13345 MecaBot encoder has 1440 ticks per rotation, wheel has 48mm diameter

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalCountPerRadianFile = AppUtil.getInstance().getSettingsFile("horizontalCountPerRadian.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry Calibration", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry Calibration", "All Init Complete, Ready to Start");
        telemetry.update();

        waitForStart();

        // Begin calibration, by pivoting the robot in place, in this setup rotate right is positive angle
        // if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){
            if(getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED); // rotate right
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2); // rotate right
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 2000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Since the orientation of encoders on each side is opposite, one of the encoder value
        needs to be reversed so that both side encoders produced positive ticks with forward movement.
        Horizontal encoder ticks may also need sign reversal, in this code clockwise rotation of robot should produce positive tick count
        THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
       */
        double verticalLeftCount = verticalLeft.getCurrentPosition();
        double verticalRightCount = -verticalRight.getCurrentPosition();
        double horizontalCount = horizontal.getCurrentPosition();

        // The Robot pivoted around its own center for a certain angle and we recorded the encoder ticks on left and right
        // wheel base separation = sum of radius of left arc and radius of right arc around the pivot point
        // Note that: length of arc for 1 radian angle = radius of circle
        // Therefore wheel base separation = sum of left arc length and right arc length for 1 radian rotation
        double verticalEncoderTicks = Math.abs(verticalLeftCount) + (Math.abs(verticalRightCount));
        double verticalEncoderTicksPerDegree = verticalEncoderTicks/angle;
        double verticalEncoderTicksPerRadian = (180*verticalEncoderTicks)/(Math.PI*angle);
        double wheelBaseSeparationInches = verticalEncoderTicksPerRadian/COUNTS_PER_INCH;

        double horizontalCountPerRadian = Math.abs(horizontalCount)/Math.toRadians(angle);

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparationInches));
        ReadWriteFile.writeFile(horizontalCountPerRadianFile, String.valueOf(horizontalCountPerRadian));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation (inches)", wheelBaseSeparationInches);
            telemetry.addData("Horizontal Encoder Ticks per Radian", horizontalCountPerRadian);

            //Display raw values
            telemetry.addData("IMU Angle (degrees)", angle);
            telemetry.addData("Vertical Left Position", verticalLeftCount);
            telemetry.addData("Vertical Right Position", verticalRightCount);
            telemetry.addData("Horizontal Position", horizontalCount);
            telemetry.addData("Vertical Encoder Ticks per Degree", verticalEncoderTicksPerDegree);
            telemetry.addData("Vertical Encoder Ticks per Radian", verticalEncoderTicksPerRadian);

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // THIS WILL CHANGE FOR EACH ROBOT AND NEED TO BE UPDATED HERE
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Odometry Calibration", "Hardware Map Init Complete");
        telemetry.update();

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * This code assumes that clockwise rotation is positive angle, thus reverse sign of value returned by IMU
     * @return the angle of the robot
     */
    private double getZAngle(){

        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);
    }

}

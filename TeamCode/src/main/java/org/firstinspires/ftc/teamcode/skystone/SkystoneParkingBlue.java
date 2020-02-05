package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLU Park Outside Lane", group="QT")
public class SkystoneParkingBlue extends SkystoneAutoBase {

    @Override
    public String getColorString() { return "BLU"; }

    @Override
    public void setOdometryStartingPosition() {
        // Starting postion assumption:  NEAR BUILD ZONE
        // Robot is 18x18 square, robot position (x,y) is center of the robot
        // starting position is against the wall (y=0), hence robot center is y = 9 inches
        // Robot (BLUE:left | RED:right) side is exactly on the 1st and 2nd tile line (47 inches from center)
        // Robot is facing the quarry (+ve X-axis on BLUE side) hence orientation is zero degrees
        globalPosition.initGlobalPosition(-14.0, 9.0, 0.0);

    }

    @Override
    public void runOpMode() {

        aColor = FieldSkystone.AllianceColor.BLUE;

        // initialize the robot hardware, navigation, IMU, Odometry and Telemetry display
        initializeOpMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This OpMode only Parks under skybridge
        parkAtOutsideLane();

        // Don't exit, wait for user (driver presses STOP)
        waitForStop();

    }
}

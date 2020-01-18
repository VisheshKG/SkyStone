package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.skystone.FieldSkystone.AllianceColor;

@Autonomous(name="BLUE Full Auto", group="QT")

public class SkystoneAutoBlue extends SkystoneAutoBase {

    @Override
    public ColorSensor chooseColorSensorForSkystone() {
        return robot.leftColorSensor;
    }

    @Override
    public void setOdometryStartingPosition() {

        // Robot is 18x18 square, robot position (x,y) is center of the robot
        // starting position is against the wall (y=0), hence robot center is y = 9 inches
        // facing the quarry (+ve X-axis on BLUE side) hence orientation is zero degrees
        // touching the floor tile edge on stone quarry side (hence X = 23 - 9 = 14 inches)
        globalPosition.initGlobalPosition(14.0, 9.0, 0.0);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        aColor = AllianceColor.BLUE;

        // initialize the robot hardware, navigation, IMU, Odometry and Telemetry display
        initializeOpMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This OpMode does all the tasks in the autonomous period of 30 seconds
        runFullAutoProgram();

        // Don't exit, wait for user (driver presses STOP)
        waitForStop();
    }

}
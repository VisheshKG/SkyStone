package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.MecaBot;

@TeleOp(name = "Test Odometry Mecabot", group = "Test")
public class TestMecabotOdometry extends LinearOpMode {

    private MecaBot robot = new MecaBot();   // Use a Mecabot's hardware
    private OdometryGlobalPosition robotGlobalPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.initOdometry();
        robotGlobalPosition = robot.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robotGlobalPosition.getXCount() / robot.ODOMETRY_COUNT_PER_INCH);
            telemetry.addData("Y Position", robotGlobalPosition.getYCount() / robot.ODOMETRY_COUNT_PER_INCH);
            telemetry.addData("Orientation (Degrees)", robotGlobalPosition.getOrientationDegrees());

            telemetry.addData("Vertical left encoder", robotGlobalPosition.getVerticalLeftCount());
            telemetry.addData("Vertical right encoder", robotGlobalPosition.getVerticalRightCount());
            telemetry.addData("horizontal encoder", robotGlobalPosition.getHorizontalCount());

            telemetry.update();
        }

        //Stop the thread
        robotGlobalPosition.stop();

    }
}

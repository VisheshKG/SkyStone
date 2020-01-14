package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

@TeleOp(name = "Test Odometry Mecabot", group = "Test")
public class TestMecabotOdometry extends LinearOpMode {

    private MecaBot robot;
    private MecaBotMove nav;
    private OdometryGlobalPosition robotGlobalPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecaBot();
        robot.init(hardwareMap);

        nav = new MecaBotMove(this, robot);
        robotGlobalPosition = nav.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // start odometry reading calculations before any driving begins
        nav.startOdometry();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robotGlobalPosition.getXinches());
            telemetry.addData("Y Position", robotGlobalPosition.getYinches());
            telemetry.addData("Orientation (Degrees)", robotGlobalPosition.getOrientationDegrees());

            telemetry.addData("Vertical left encoder", robotGlobalPosition.getVerticalLeftCount());
            telemetry.addData("Vertical right encoder", robotGlobalPosition.getVerticalRightCount());
            telemetry.addData("horizontal encoder", robotGlobalPosition.getHorizontalCount());

            telemetry.update();
        }

        //Stop the thread
        nav.stopOdometry();

    }
}

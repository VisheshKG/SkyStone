package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalPosition;
import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;
import org.firstinspires.ftc.teamcode.skystone.FieldSkystone;

@TeleOp(name = "Test Odometry move methods", group = "Test")
public class TestMecabotOdometry extends LinearOpMode {

    private MecaBot robot;
    private MecaBotMove nav;
    private OdometryGlobalPosition globalPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MecaBot();
        robot.init(hardwareMap);
        robot.initIMU();

        nav = new MecaBotMove(this, robot);
        globalPosition = nav.getPosition();
        globalPosition.initGlobalPosition(14.0, 9.0, 0.0);

        telemetry.addData("Status", "Initialized");

        telemetry.addLine("Global Position ")
                .addData("X", "%3.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getXinches();
                    }
                })
                .addData("Y", "%2.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getYinches();
                    }
                })
                .addData("Angle", "%4.2f", new Func<Double>() {
                    @Override public Double value() {
                        return globalPosition.getOrientationDegrees();
                    }
                });
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // start odometry reading calculations before any driving begins
        nav.startOdometry();

        nav.goToPosition(33.0, 33);
        sleep(2000);
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_NEG_Y_AXIS);
        sleep(2000);
        nav.goToXPosition(23.5);
        sleep(200);
        nav.goToYPosition(23.5);
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_NEG_Y_AXIS);

        nav.odometryMoveDistance(-12.0, MecaBotMove.DriveType.TANK, MecaBotMove.DRIVE_SPEED_SLOW);
        sleep(200);
        nav.odometryMoveDistance(-12.0, MecaBotMove.DriveType.MECANUM, MecaBotMove.DRIVE_SPEED_SLOW);
        sleep(200);
        nav.odometryMoveDistance(+12.0, MecaBotMove.DriveType.TANK, MecaBotMove.DRIVE_SPEED_SLOW);
        sleep(200);
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_NEG_X_AXIS);
        sleep(10000);
        nav.goToPosition(14, 24);
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS);
        nav.odometryMoveDistance(+15, MecaBotMove.DriveType.MECANUM, MecaBotMove.DRIVE_SPEED_SLOW);
        nav.odometryRotateToHeading(FieldSkystone.ANGLE_POS_X_AXIS);


//        nav.gyroRotateToHeading(8, 0.2);
//        sleep(2000);
//        nav.gyroRotateToHeading(80, 0.3);
//        sleep(2000);
//        nav.gyroRotateToHeading(-90, 0.2);
//        sleep(2000);
//        nav.gyroRotateToHeading(0, 0.25);

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){

//            telemetry.addData("Vertical left encoder", globalPosition.getVerticalLeftCount());
//            telemetry.addData("Vertical right encoder", globalPosition.getVerticalRightCount());
//            telemetry.addData("horizontal encoder", globalPosition.getHorizontalCount());

            telemetry.update();
        }

        //Stop the thread
        nav.stopOdometry();

    }
}

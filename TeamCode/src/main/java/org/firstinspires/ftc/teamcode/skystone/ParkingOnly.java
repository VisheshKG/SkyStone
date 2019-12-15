package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

@Autonomous(name = "Parking Only")
//@Disabled
public class ParkingOnly extends LinearOpMode {

    MecaBot robot = new MecaBot();
    MecaBotMove nav = new MecaBotMove(this, robot);

    public void runOpMode() {
        robot.init(this.hardwareMap);
        telemetry.setAutoClear(false);
        FieldSkystone.initRobotStartX();
        double curX= FieldSkystone.robotStartX;
        double curY= FieldSkystone.robotStartY;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        ElapsedTime opmodeRunTime = new ElapsedTime();

        while (opmodeRunTime.seconds() < FieldSkystone.delayParkingBySeconds){
            sleep(500);
            telemetry.addData("Waiting Time (seconds)", "%.1f seconds", opmodeRunTime.seconds());
        }
        telemetry.addData("start X Y", "%.1f %.1f", curX,curY);
        telemetry.update();

        nav.moveForwardBack(FieldSkystone.driveToPark);
/*
        boolean headXpositive=!FieldSkystone.BLUESIDE;
        nav.goPark(curX,curY,FieldSkystone.PARK_INSIDE,headXpositive);

        while (opModeIsActive()){

        }

 */
    }

}

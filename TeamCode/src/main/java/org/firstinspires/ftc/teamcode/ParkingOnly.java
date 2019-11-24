package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking Only")
//@Disabled
public class ParkingOnly extends LinearOpMode {

    MecaBot robot = new MecaBot();
    MecaBotMove nav = new MecaBotMove(this, robot);

    public void runOpMode() {
        robot.init(this.hardwareMap);
        telemetry.setAutoClear(false);
        fieldConfiguration.initRobotStartX();
        double curX=fieldConfiguration.robotStartX;
        double curY=fieldConfiguration.robotStartY;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        ElapsedTime opmodeRunTime = new ElapsedTime();

        while (opmodeRunTime.seconds() < fieldConfiguration.delayParkingBySeconds){
            sleep(500);
            telemetry.addData("Waiting Time (seconds)", "%.1f seconds", opmodeRunTime.seconds());
        }
        telemetry.addData("start X Y", "%.1f %.1f", curX,curY);
        telemetry.update();

        nav.moveForwardBack(fieldConfiguration.driveToPark);
/*
        boolean headXpositive=!fieldConfiguration.BLUESIDE;
        nav.goPark(curX,curY,fieldConfiguration.PARK_INSIDE,headXpositive);

        while (opModeIsActive()){

        }

 */
    }

}

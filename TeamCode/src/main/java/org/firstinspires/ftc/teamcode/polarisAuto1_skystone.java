package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto1_skystone")
//@Disabled
public class polarisAuto1_skystone extends LinearOpMode {
    //Configure -----------

    // Blue Alliance or Red Alliance
    private static boolean BLUESIDE =true;

    // all in inches
    private static int stoneOffset= 12;
    private static int robotLength=18;
    private static int stonePlacementY=47;
    private static int robotStartingX=36;

    // ----------------
    private boolean DR_FORWARD = true;
    private boolean DR_BACKWARD = false;


    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    MecaBotMove nav = new MecaBotMove(this, robot);

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.init(this.hardwareMap);

        /*
        nav.initVuforia(this, robot);
        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();
*/
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        //robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            nav.testMove();
/*
            moveToScanStone();

            if (findSkyStone()) {
                nav.grabTheStone();
                break;
            }
            */
            break;
        }
    }

    //Assume to start on stone side
    private void moveToScanStone(){
        double Ydistance = stonePlacementY-stoneOffset-robotLength/2;
        telemetry.addData("Wheel Forward Requested:",Ydistance);
        telemetry.update();
        sleep(1000);
        nav.moveBackward(Ydistance);
        double inchMoved = nav.getWheelMoveInches();
        telemetry.addData("Wheel Forward Actual:",inchMoved);
        telemetry.update();
    }

    private boolean findSkyStone(){
        boolean stonefound=false;

        double xdistance=49-robotStartingX;   //stone placed at 49 inches; robot starting 36
        //call vuforia to find stone, start scanning from bridge end to wall
        // if robot on blue side, it moves left first. 2nd parameter, true to move robot left
        nav.moveLeftRight(xdistance, BLUESIDE);

        return stonefound;
    }
}
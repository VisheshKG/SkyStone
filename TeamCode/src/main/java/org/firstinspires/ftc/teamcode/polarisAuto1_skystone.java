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
    private static final float mmPerInch = 25.4f;


    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    // todo: restore
    //  MecaBotMove nav = new MecaBotMove(this, robot);
    polarisVuforiaUtil vUtil= new polarisVuforiaUtil(this);

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        //todo: restore
        // robot.init(this.hardwareMap);


        vUtil.initVuforia();
        // Activate Vuforia (this takes a few seconds)
        vUtil.activateTracking();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        //robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //while (opModeIsActive()) {
        while (!isStopRequested()) {
            //nav.testMove();
            findSkyStone();

/*
            moveToScanStone();

            if (findSkyStone()) {
                nav.grabTheStone();
                break;
            }
            */
        }
        vUtil.stopTracking();
    }
double inchClosetoScan=12.25;

    //Assume to start on stone side
    private void moveToScanStone(){
        //double Ydistance = stonePlacementY-stoneOffset-robotLength/2;
        double Ydistance = inchClosetoScan;
        telemetry.addData("Wheel Forward Requested:",Ydistance);
        telemetry.update();
        sleep(2000);
        //todo: restore
        // nav.moveRight(Ydistance);
        //double inchMoved = nav.getWheelMoveInches();
        //telemetry.addData("Wheel Forward Actual:",inchMoved);
    }

    float stoneDistanceMargin = 25; //in mm
    private boolean findSkyStone(){
        double smallStep = 75;  //mm
        boolean stonefound=false;
        float x;
        float y;
        float z;

        //double xdistance=49-robotStartingX;   //stone placed at 49 inches; robot starting 36
        //call vuforia to find stone, start scanning from bridge end to wall
        // if robot on blue side, it moves left first. 2nd parameter, true to move robot left

        while (!stonefound ) {
            if (vUtil.skystoneIsVisible()){
                stonefound=true;
            }
            //nav.moveForward(3);
            sleep(1000);
        }

        if (stonefound){
            x=vUtil.getRobotX();
            y=vUtil.getRobotY();
            z=vUtil.getRobotZ();
            float xinch=x/mmPerInch;
            float yinch=y/mmPerInch;
            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",x,y,z);
            //telemetry.update();
            if (y < 0) {
                telemetry.addData("---Stone on Left", yinch);
                if (Math.abs(y) > stoneDistanceMargin) {
                    telemetry.addData("Move Left", yinch);
                    //vishesh_move("LEFT","SLOW");
                } else {   //NOTE: This is where you grab the stone and move to load.
                    telemetry.addData("Stop", yinch);
                    //vishesh_move("STOP","SLOW");
                    //vishesh_grabStone("GET_STONE");
                }
            } else {
                telemetry.addData("Stone on Right-----", yinch);
                if (Math.abs(y) > stoneDistanceMargin) {
                    telemetry.addData("Move Right", yinch);
                    //vishesh_move("RIGHT","SLOW",yinch);
                } else {   //NOTE: This is where you grab the stone and move to load.
                    telemetry.addData("Stop", yinch);
                    //vishesh_move("STOP","SLOW");
                    //vishesh_grabStone("GET_STONE");
                }
            }
            //move close to stone
            //todo: nav.moveRightBlue or left Red by xinch-offset
            telemetry.addData("MoveToStone", xinch);
        }else{
            telemetry.addData("<<findSkyStone:Stone not found","222");
        }
        telemetry.update();
        //sleep(1000);

        return stonefound;
    }
}
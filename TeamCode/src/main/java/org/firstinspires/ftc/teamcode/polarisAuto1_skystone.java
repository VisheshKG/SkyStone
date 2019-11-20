package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto1_skystone")
//@Disabled
public class polarisAuto1_skystone extends LinearOpMode {
    //Configure -----------

    // Blue Alliance or Red Alliance
    private static boolean BLUESIDE =true;

    // all in inches
    private static double scanInterval=17;
    //3" clearance on both side of robot between bridge poll and the parked robot
    // 4.5= 46-18-1/2 *22.25
    private static double backDistToCtrBridge=4.5;
    private static double closeToStone=1.0;
    private static double inchClosetoScan=10.75;// 19" away from stone=47-17.25-19



    //eye placed 11 inch from far side of viewable stone
    // 18.5 inch off image means eye can see 3 stones 24 inches

    private static int stoneOffset= 1;
    private static int robotLength=18;
    private static int stonePlacementY=47;
    private static int robotStartingX=36;

    // ----------------
    private static final float mmPerInch = 25.4f;


    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    MecaBotMove nav = new MecaBotMove(this, robot);
    polarisVuforiaUtil vUtil= new polarisVuforiaUtil(this);

    @Override
    public void runOpMode() {
        ElapsedTime opmodeRunTime = new ElapsedTime();

        // Initialize the robot and navigation
        //todo: restore
        robot.init(this.hardwareMap);


        vUtil.initVuforia();
        vUtil.activateTracking();  //takes a few seconds


        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");
            telemetry.update();
        }

        //nav.testMove();
        //todo: restore
        //moveCloserToStone();
        telemetry.addData(">Start Look for Sky Stone", "%.1f seconds", opmodeRunTime.seconds());
        if (findSkyStone()){
            moveToStone();
            //grabStone and ship out
        }else{  //todo: go parking
            telemetry.addData("<<Stone not found","NOOOOOOOOOOOO!");
        }

        telemetry.addData("<End Look for Stone", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.update();
        while (!isStopRequested()) {  //just loop
        }
        vUtil.stopTracking();
    }

    //Assume to start on stone side
    private void moveCloserToStone(){
        telemetry.addData("Move to within Vuforia Range:",inchClosetoScan);
        nav.moveRight(inchClosetoScan);
    }

    float stoneDistanceMargin = 25; //in mm

    private boolean findSkyStone(){
        double smallStep = 75;  //mm
        boolean stonefound=false;


        //double xdistance=49-robotStartingX;   //stone placed at 49 inches; robot starting 36
        //call vuforia to find stone, start scanning from bridge end to wall
        // if robot on blue side, it moves left first. 2nd parameter, true to move robot left

        int ct=0;
        int numLooks=20;
        while (!stonefound){   //todo: need to time out
            if (vUtil.skystoneIsVisible()){
                stonefound=true;
            } else {
                sleep(300);
                ct=ct+1;
                if (ct > (2* numLooks)){
                    break;
                }else if (ct == numLooks) {
                    telemetry.addData("debug Move to new location and scan ct=",ct);
                    nav.moveForward(scanInterval);
                }
            }
        }
        telemetry.addData("debug total view count=",ct);

        return stonefound;
    }

    private void moveToStone(){
        float x;
        float y;

        x=vUtil.getRobotX();
        y=vUtil.getRobotY();
        float xinch=x/mmPerInch;
        float yinch=y/mmPerInch;
        //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",x,y,z);
        //telemetry.update();

        if (Math.abs(y) > stoneDistanceMargin) {
            telemetry.addData("Too off center=", yinch);
            if (y < 0) {
                telemetry.addData("<<<<<Stone on Left-Move Left", yinch);
                //todo: blue side left is forward
                nav.moveForward(yinch);
            } else {   //NOTE: This is where you grab the stone and move to load.
                telemetry.addData(">>>>>Stone on Right-Move Right", yinch);
                nav.moveBackward(yinch);
            }
        } else {
            telemetry.addData("Stone is Centered", yinch);
            //vishesh_move("STOP","SLOW");
        }
        //move close to stone
        //todo: nav.moveRightBlue or left Red by xinch-offset
        telemetry.addData("MoveToStone", xinch);
        double adv=xinch-closeToStone;  //include vuforia overshot of 1 inch
        nav.moveRight(adv);
    }
}
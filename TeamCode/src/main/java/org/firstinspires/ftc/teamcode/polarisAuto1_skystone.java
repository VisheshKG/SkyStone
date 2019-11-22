package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto1_skystone")
//@Disabled
public class polarisAuto1_skystone extends LinearOpMode {
    //--------Configure -----------

    // Blue Alliance or Red Alliance
    private static boolean BLUESIDE =true;

    //--------End of Configure
    double LOW_SPEED=0.5;
    double HIGH_SPEED=0.5;

    // Field placement measures all in inches
    private static double robotStartX=49.0-12-2;
    private static double robotStartY=17.25;
    private static double scanInterval=17;  //2 scan 17 apart; 3 scan
    //3" clearance on both side of robot between bridge poll and the parked robot
    // 4.5= 46-18-1/2 *22.25
    private static double backDistToCtrBridge=4.5;
    private static double closeToStone=2.0;
    private static double inchClosetoScan=10.75;// 19" away from stone=47-17.25-19

    //eye placed 11 inch from far side of viewable stone
    // 18.5 inch off image means eye can see 3 stones 24 inches
    //private static int stonePlacementY=47;
    private static int stonePlacementY=18;

    //current location: origin is at red/blue wall center with x pointing to stone side and y to center of field
    private static double curX=robotStartX;
    private static double curY=robotStartY;


    // ----------------
    private static final float mmPerInch = 25.4f;


    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    MecaBotMove nav = new MecaBotMove(this, robot);
    polarisVuforiaUtil vUtil= new polarisVuforiaUtil(this);

    @Override
    public void runOpMode() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        //todo: debug restore
        curX=0.0;
        curY=0.0;

        // Initialize the robot and navigation
        //todo: restore
        robot.init(this.hardwareMap);

        vUtil.initVuforia();
        vUtil.activateTracking();  //takes a few seconds
        telemetry.setAutoClear(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //nav.testMove();
        //todo: restore
        //moveCloserToStone();
        telemetry.addData("{curX, curY} =", "%.2f, %.2f",curX,curY);

        telemetry.addData(">Start Look for Sky Stone", "%.1f seconds", opmodeRunTime.seconds());
        if (findSkyStone()){
            moveToStone();
            telemetry.addData("Grab Stone!!!!!!!","none");
            grabStoneNship();
            //todo: nav.grabStone
        }else{
            telemetry.addData("<<Stone not found","NOOOOOOOOOOOO!");
        }
        telemetry.addData("Go parking","none");
        //todo: goParking(position);
        telemetry.addData("<End Look for Stone", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.update();
        while (!isStopRequested()) {  //just loop
        }
        vUtil.stopTracking();
    }

    //Assume to start on stone side, move to scan range
    private void moveCloserToStone(){
        double moveInches;
        if (BLUESIDE){
            //move right is negative
            moveInches=-inchClosetoScan;
        }else{
            moveInches=inchClosetoScan;
        }
        curY=curY+inchClosetoScan;
        telemetry.addData("Move to within Vuforia Range:",moveInches);
        nav.setSpeedWheel(HIGH_SPEED);
        nav.moveLeftRight(inchClosetoScan);
    }

    float stoneDistanceMargin = 25; //in mm

    private boolean findSkyStone(){
        double smallStep = 75;  //mm
        boolean stonefound=false;


        //double xdistance=49-robotStartingX;   //stone placed at 49 inches; robot starting 36
        //call vuforia to find stone, start scanning from bridge end to wall
        // if robot on blue side, it moves left first. 2nd parameter, true to move robot left

        int ct=0;
        int numLooks=20;  //controls how long robot looks for target at each location
        while (!stonefound){   //todo: need to time out
            if (vUtil.skystoneIsVisible()){
                stonefound=true;
            } else {
                //sleep(300);
                ct=ct+1;
                if (ct > (3* numLooks)){
                    break;
                }else if (ct == numLooks) {  //time is up, move to new location to find stone
                    telemetry.addData("debug Move to new location and scan ct=",ct);
                    //move back in negative
                    double inchMove=BLUESIDE?-scanInterval:scanInterval;
                    nav.moveForwardBack(inchMove);
                    curX=curX+scanInterval;
                    telemetry.addData("222 Field Current Position {x y}=","%.2f  %.2f", curX,curY);
                }
            }
        }
        telemetry.addData("debug total view count=",ct);

        return stonefound;
    }

    private void moveToStone(){
        float x;
        float y;

        //Vuforia x, y: x is point out screen y pointing to right of screen in landscape;
        x=vUtil.getRobotX();
        y=vUtil.getRobotY();
        float xinch=x/mmPerInch;
        float yinch=y/mmPerInch;
        telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f",x,y);
        telemetry.update();

        nav.setSpeedWheel(LOW_SPEED);

        if (Math.abs(y) > stoneDistanceMargin) {
            telemetry.addData("Too off center=", yinch);
            if (y < 0) {
                telemetry.addData("<<<<<Stone on Left-Move Left", yinch);
                //todo: blue side left is forward, pass positive inch
                nav.moveForwardBack(-yinch);
            } else {   //NOTE: This is where you grab the stone and move to load.
                telemetry.addData(">>>>>Stone on Right-Move Right", yinch);
                nav.moveForwardBack(-yinch);
            }
            curX=curX+yinch;
        } else {
            telemetry.addData("Stone is Centered", yinch);
            //vishesh_move("STOP","SLOW");
        }
        //advance to move close to stone for grabbing
        telemetry.addData("MoveToStone", xinch);
        double adv=Math.abs(xinch)-closeToStone;  //include vuforia overshot of 1 inch
        nav.moveLeftRight(-adv);
        curY=curY+adv;
        telemetry.addData("Field Current Position {x y}=","%.2f  %.2f", curX,curY);
    }

    private void grabStoneNship(){
        telemetry.addData("Debug:GRABBBBBBBBBBB", "None");
        //nav.grabTheStone();
        nav.moveLeftRight(backDistToCtrBridge);  //back off from stone to location safe to cross bridge
        curY=curY-backDistToCtrBridge;
        // move across bridge from x=72-5=67 to x=72-(49-4-7.5)=36
        deliverStone();
        //nav.releaseTheStone();
    }

    //robot right back wheel corner is the dot for robot location
    private void deliverStone(){
        //with stone robot width is 22.25", bridge is 46" wide, 27" space to pass
        double margin=3;
        double targetY=stonePlacementY-margin;  //need to clear the bridge
        double targetX=BLUESIDE?-3:-3-17.25;

        //todo: test
        targetX=-10;
        targetY=0;
        boolean headingX=!BLUESIDE;
        telemetry.addData("Before Drop-----{x y}=","%.2f  %.2f", curX,curY);
        nav.moveYX(targetX,targetY,curX,curY,headingX);
        telemetry.addData("Stone Drop-----{x y}=","%.2f  %.2f", curX,curY);
    }
}
package org.firstinspires.ftc.teamcode.recognition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

@Autonomous(name = "VuforiaAuto1Skystone", group = "Concept")
//@Disabled
public class VuforiaAuto1Skystone extends LinearOpMode {
    //--------Configure -----------

    // Blue Alliance or Red Alliance
    private static boolean BLUESIDE = MecaBotMove.BLUESIDE;

    //--------End of Configure
    double LOW_SPEED=0.3;
    double HIGH_SPEED=0.5;

    // Field placement measures all in inches

    //Robot camera(eye) and arm placement
    private static final double eyeFromRBorigin=9.0;  //distance from camera eye to robot right back corner
    private static final double sideArmFromRBorigin = 7.0;
    private static final double eyeFromArm=2.0;

    //robot origin is right back corner of robot
    //robot start position: put the eye at center of 2nd stone
    private static double robotStartX= MecaBotMove.robotStartX;
    private static double robotStartY= MecaBotMove.robotStartY;       //right back corner of robot
    //3" clearance on both side of robot between bridge poll and the parked robot
    // 4.5= 46-18-1/2 *22.25
    private static double backDistToCtrBridge=4.5;

    private static double inchClosetoScan= 14.75; //*** 15" away from stone;increase to close in

    //eye placed 11 inch from far side of viewable stone
    // 18.5 inch off image means eye can see 3 stones 24 inches
    private static int stonePlacementY=47;
    //private static int stonePlacementY=18;

    //current location: origin is at red/blue wall center with x pointing to stone side and y to center of field
    private static double curX=robotStartX;
    private static double curY=robotStartY;

    //Vuforia setting
    private static final double scanInterval=8;
    private static final double maxTimeViewStone=10;
    private static final double maxTimeViewOneStone=2;
    private static final float errForwardAdjust=6;  //***left/right adjust due to over or under drive
    private static final double closeToStone=-1;  //distance from skystone for grabbing, negative means over drive


    // ----------------
    private static final float mmPerInch = 25.4f;
    ElapsedTime opmodeRunTime;


    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    MecaBotMove nav = new MecaBotMove(this, robot);
    VuforiaUtil vUtil= new VuforiaUtil(this);

    @Override
    public void runOpMode() {
        // Initialize the robot and navigation
        //todo: restore
        robot.init(this.hardwareMap);
        MecaBotMove.initRobotStartX();

        vUtil.initVuforia();
        vUtil.activateTracking();  //takes a few seconds
        telemetry.setAutoClear(false);
        curX= MecaBotMove.robotStartX;
        curY= MecaBotMove.robotStartY;
        String side="RED";
        if (BLUESIDE){ side="BLUE"; }
        telemetry.addData(" ",side);
        telemetry.addData("{curX, curY} =", "%.2f, %.2f",curX,curY);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        opmodeRunTime = new ElapsedTime();

        //nav.testMove();
        //todo: restore
        moveToScanRange();
        telemetry.addData("{curX, curY} =", "%.2f, %.2f",curX,curY);

        telemetry.addData(">Start Look for Sky Stone", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.update();
        if (findSkyStone()){
            moveToStone();
            telemetry.addData("Grab Stone!!!!!!!","none");
            grabStoneNship();
            //todo: nav.grabStone
        }else{
            telemetry.addData("<<Stone not found","NOOOOOOOOOOOO!");
        }
        telemetry.addData("<End Look for Stone", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.update();
        vUtil.stopTracking();
        while (!isStopRequested()) {  //just loop
        }

    }

    //Assume to start on stone side, move to scan range
    private void moveToScanRange(){
        double moveInches;
        moveInches=-inchClosetoScan;   //advance to arm side
        /*
        if (BLUESIDE){
            //move right is negative
            moveInches=-inchClosetoScan;
        }else{
            moveInches=inchClosetoScan;
        }
         */
        curY=curY+inchClosetoScan;
        telemetry.addData("Move to within Vuforia Range:",moveInches);
        nav.encoderMoveLeftRight(moveInches, HIGH_SPEED);
    }


    private boolean findSkyStone(){
        boolean stonefound=false;

        //double xdistance=49-robotStartingX;   //stone placed at 49 inches; robot starting 36
        //call vuforia to find stone, start scanning from bridge end to wall
        // if robot on blue side, it moves left first. 2nd parameter, true to move robot left
        double curSeconds = opmodeRunTime.seconds();
        double lastSeconds = curSeconds;
        double deltaTime;

        int maxCt;
        double limitX=2;
        if (BLUESIDE){
            maxCt=3;
            limitX=52.0;  //half field - space from wall=72-10
        }else{
            maxCt=2;
            limitX=48;  //half field - 3 stone over=72-20
        }

        int ct=0;  //Count # moves
        while (!stonefound){   //todo: need to time out
            if (vUtil.skystoneIsVisible()){
                stonefound=true;
            } else {
                //ct=ct+1;
                curSeconds = opmodeRunTime.seconds(); //update current time
                if (curSeconds > maxTimeViewStone){
                    telemetry.addData("Max time stone scan reached ", "%.1f", opmodeRunTime.seconds());
                    telemetry.update();
                    break;
                }
                deltaTime=curSeconds - lastSeconds;
                /*
                if (ct > (2* numLooks)){   //control the number of looking position
                    break;
                }else if (ct == numLooks) {  //time is up, move to new location to find stone
                                    telemetry.addData("222 Field Current Position {x y}=","%.2f  %.2f", curX,curY);

                 */
                if (deltaTime > maxTimeViewOneStone){
                    ct=ct+1;
                    /*
                    if (ct > maxCt) {
                        telemetry.addData("Max Scan Count Reached", ct);
                        telemetry.update();
                        break;
                    }
                     */
                    double nextX=curX+scanInterval;
                    telemetry.addData("Ready to Move to next view point curX=", nextX);
                    if (nextX > limitX){
                        telemetry.addData("Max X Reached limitX=", limitX);
                        telemetry.update();
                        break;
                    }else {
                        telemetry.addData("====Move to new location to scan ct (seconds)", "%d %.1f", ct, curSeconds);
                        telemetry.update();
                        double inchMove = BLUESIDE ? -scanInterval : scanInterval;
                        nav.encoderMoveForwardBack(inchMove);   //Move
                        curX = curX + scanInterval;  //track coordinate
                        lastSeconds = curSeconds;   //reset per stone view time
                        telemetry.addData("----Field Current Position {x y}=", "%.2f  %.2f", curX, curY);
                        telemetry.addData("111111 view stone", ct);
                        telemetry.update();
                    }

                }
            }
        }
        telemetry.addData("!!!!!!!!!!!!!!debug total view count=",ct);
        telemetry.update();

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

        //nav.setSpeedWheel(LOW_SPEED);
        if (y>0){     //test shows a drift to right of stone
            yinch=yinch- errForwardAdjust;  //move less to right stone
        }else{
            yinch=yinch- errForwardAdjust;
        }
        float stoneDistanceMargin = 1; //Vuforia stone center margin
        if (Math.abs(yinch) > stoneDistanceMargin) {
            telemetry.addData("Too off center=", yinch);
            if (y < 0) {
                telemetry.addData("<<<<<Stone on Left-Move Left", yinch);
                //todo: blue side left is forward, pass positive inch
                nav.encoderMoveForwardBack(-yinch);
            } else {   //NOTE: This is where you grab the stone and move to load.
                telemetry.addData(">>>>>Stone on Right-Move Right", yinch);
                nav.encoderMoveForwardBack(-yinch);
            }
            curX=curX-yinch;     //fixed the bug on sign
        } else {
            telemetry.addData("Stone is Centered", yinch);
            //vishesh_move("STOP","SLOW");
        }
        //advance to move close to stone for grabbing
        telemetry.addData("MoveToStone", xinch);
        double adv=Math.abs(xinch)- closeToStone;  //include vuforia overshot of 1 inch
        nav.encoderMoveLeftRight(-adv);
        curY=curY+adv;
        telemetry.addData("Field Current Position {x y}=","%.2f  %.2f", curX,curY);
    }

    private void grabStoneNship(){
        telemetry.addData("GRAB STONE", "None");
        robot.dropCapstoneClips();
        sleep(500);
        nav.encoderMoveLeftRight(backDistToCtrBridge, LOW_SPEED);  //back off from stone to location safe to cross bridge
        curY=curY-backDistToCtrBridge;
        // move across bridge from x=72-5=67 to x=72-(49-4-7.5)=36
        deliverStone();
        robot.resetCapstoneClips();
    }

    //robot right back wheel corner is the dot for robot location
    private void deliverStone(){
        //with stone robot width is 22.25", bridge is 46" wide, 27" space to pass
        double margin=3;
        double targetY=stonePlacementY-margin;  //need to clear the bridge
        double targetX=BLUESIDE?-6:-6-17.25;   //6 inch over bridge

        boolean headingX=!BLUESIDE;
        telemetry.addData("Before Drop-----{x y}=","%.2f  %.2f", curX,curY);
        nav.moveYX(targetX,targetY,curX,curY,headingX);
        curX=targetX;
        curY=targetY;
        telemetry.addData("Stone Drop-----{x y}=","%.2f  %.2f", curX,curY);
    }
}
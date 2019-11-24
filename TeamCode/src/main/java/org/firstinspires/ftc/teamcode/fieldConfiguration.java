package org.firstinspires.ftc.teamcode;


/**  Main Configuration for Sky Stone Challenge game field
 **  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
 **  with Y pointing to the middle of the field and X pointing to the stone side.
 */

public class fieldConfiguration {
    public static final boolean BLUESIDE =false;      //if red side, set it to false
    public static final boolean PARK_INSIDE =false;   //false for parking along wall
    public static final boolean START_STONE_SIDE=false;  //true if start at stone side

    public static double robotStartX= 41;      // robot origin aline with image right
    public static double robotStartY=17.25;       //right back corner of robot

    //Vuforia setting
    public static final double scanIntervalDistance=10;
    public static final double maxTimeViewStone=5;
    public static final double maxTimeViewOneStone=2;
    public static final double inchClosetoScan=12.75; // 17" away from stone=47-17.25-19

    //parking
    public static final double delayParkingBySeconds = 0;
    public static final double driveToPark=32;

    public static final double parkingMarginL=1;   //leave space on left side of robot at parking
    public static final double parkingMarginR=4;   //leave space on right/arm side of robot at parking
    public static final double bridgeY=46;

    //Robot measures
    public static final double robotLength=22;
    public static final double robotWidth=17.25;

    public static void initRobotStartX(){
        if (START_STONE_SIDE) {
            if (BLUESIDE) {
                robotStartX = 41.5;   //blue stone side: align with wall image right edge
            } else {
                robotStartX = 24;   //red stone side: align with tile edge
            }
        }else{     //foundation side
            if (BLUESIDE) {
                robotStartX = -24;   //blue foundation side: align tile
            } else {
                robotStartX = -24-17;   //red foundation side: align with tile edge
            }
        }
    }

}

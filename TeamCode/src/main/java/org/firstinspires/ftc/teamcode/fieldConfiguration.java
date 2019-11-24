package org.firstinspires.ftc.teamcode;


/**  Main Configuration for Sky Stone Challenge game field
 **  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
 **  with Y pointing to the middle of the field and X pointing to the stone side.
 */

public class fieldConfiguration {
    public static final boolean BLUESIDE =false;      //if red side, set it to false
    public static final boolean PARK_INSIDE =true;
    public static final boolean START_STONE_SIDE=true;  //true if start at stone side

    // field distances between objects
    static final double     TILE_SIZE                   = 24.0;
    static final double     SIDE_WALL_TO_FOUNDATION     = 47.25;
    static final double     BUILD_WALL_TO_FOUNDATION    = 4.0;
    static final double     FOUNDATION_LENGTH           = 37.5;
    static final double     FOUNDATION_WIDTH            = 18.5;

    public static double robotStartX= 41;      // robot origin aline with image right
    public static double robotStartY=17.25;       //right back corner of robot

    //Vuforia setting
    public static final double scanIntervalDistance=8;
    public static final double maxTimeViewStone=10;
    public static final double maxTimeViewOneStone=2;
    public static final double inchClosetoScan=14.75; //*** 15" away from stone;increase to close in
    public static final float errForwardAdjust=6;  //***left/right adjust due to over or under drive
    //red side drift right by 5 inches including eye offset; move forward to compensate
    public static final double leftRightMultiple=1.1;  //multiply this by left right movements
    public static final double closeToStone=-1;  //distance from skystone for grabbing, negative means over drive

    //parking
    public static final double delayParkingBySeconds = 0;
    public static final double driveToPark=32;

    public static final double parkingMarginL=1;   //leave space on left side of robot at parking
    public static final double parkingMarginR=2;   //leave space on right/arm side of robot at parking
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
                robotStartX = -41;   //red foundation side: align with tile edge
            }
        }
    }

}

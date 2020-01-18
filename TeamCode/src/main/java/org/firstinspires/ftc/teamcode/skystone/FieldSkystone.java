package org.firstinspires.ftc.teamcode.skystone;


/**  Main Configuration for Sky Stone Challenge game field
 **  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
 **  with Y pointing to the middle of the field and X pointing to the stone side.
 */

public class FieldSkystone {

    // FTC Team alliance color, BLUE or RED, the field is mirror images for each side
    // therefore lot of robot movement calculations are affected by which color you are on
    enum AllianceColor { BLUE, RED}

    public static final double  ANGLE_POS_X_AXIS = 0.0;
    public static final double  ANGLE_POS_Y_AXIS = 90.0;
    public static final double  ANGLE_NEG_X_AXIS = 180.0;
    public static final double  ANGLE_NEG_Y_AXIS = -90.0;

    public static final boolean BLUESIDE =false;      //if red side, set it to false
    public static final boolean PARK_INSIDE =true;
    public static final boolean START_STONE_SIDE=true;  //true if start at stone side

    // field distances between objects
    public static final double     TILE_SIZE                   = 24.0;
    public static final double     SIDE_WALL_TO_FOUNDATION     = 47.0;
    public static final double     BUILD_WALL_TO_FOUNDATION    = 4.0;
    public static final double     FOUNDATION_LENGTH           = 37.5;
    public static final double     FOUNDATION_WIDTH            = 18.5;

    public static double robotStartX= 41;      // robot origin aline with image right
    public static double robotStartY=17.25;       //right back corner of robot

    //Vuforia setting
    public static final double scanIntervalDistance=8;
    public static final double maxTimeViewStone=10;
    public static final double maxTimeViewOneStone=2;
    public static final double inchClosetoScan=14.75; //*** 15" away from stone;increase to close in
    public static final float errForwardAdjust=6;  //***left/right adjust due to over or under drive
    //red side drift right by 5 inches including eye offset; move forward to compensate
    public static final double rightMultiple=1.07;  //multiply this to right movements

    public static final double closeToStone=-1;  //distance from skystone for grabbing, negative means over drive

    //parking
    public static final double delayParkingBySeconds = 0;
    public static final double driveToPark=-32;

    //value adjust for RED side stone pick up and return to parking
    public static final double parkingMarginL=1;   //leave space on left side of robot at parking
    public static final double parkingMarginR=0;   //leave space on right/arm side of robot at parking
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

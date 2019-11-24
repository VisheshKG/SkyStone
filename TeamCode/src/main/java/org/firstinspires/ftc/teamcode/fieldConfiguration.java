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

    public static final double delayParkingBySeconds = 0;

    //Vuforia setting
    public static final double scanIntervalDistance=10;
    public static final double maxTimeViewStone=5;
    public static final double maxTimeViewOneStone=2;

    //parking
    public static final double parkingMargin=1;   //leave space on side of robot at parking
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
                robotStartX = -48;   //blue foundation side: align tile
                robotStartY = 0;
            } else {
                robotStartX = -48;   //red foundation side: align with tile edge
            }
        }
    }

}

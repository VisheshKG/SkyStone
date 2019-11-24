package org.firstinspires.ftc.teamcode;


/**  Main Configuration for Sky Stone Challenge game field
 **  The coordinate origin is assumed to be at the alliance wall center (where bridge touch the wall)
 **  with Y pointing to the middle of the field and X pointing to the stone side.
 */

public class fieldConfiguration {
    public static final boolean BLUESIDE =false;      //if red side, set it to false
    public static final boolean PARK_INSIDE =true;
    public static final double robotStartX= 41;      // robot origin aline with image right
    public static final double robotStartY=17.25;       //right back corner of robot

    public static final double delayParkingBySeconds = 0;

    //parking
    public static final double parkingMargin=1;   //leave space on side of robot at parking
    public static final double bridgeY=46;

    //Robot measures
    public static final double robotLength=23;
    public static final double robotWidth=17.25;


}

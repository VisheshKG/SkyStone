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

    // field distances between objects
    static final double     TILE_SIZE                   = 24.0;
    static final double     SIDE_WALL_TO_FOUNDATION     = 47.0;
    static final double     BUILD_WALL_TO_FOUNDATION    = 4.0;
    static final double     FOUNDATION_LENGTH           = 37.5;
    static final double     FOUNDATION_WIDTH            = 18.5;

}

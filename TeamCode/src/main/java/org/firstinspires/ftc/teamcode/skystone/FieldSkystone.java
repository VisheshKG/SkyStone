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
    static final double     X_ORIGIN                    = 0.0;
    static final double     HALF_LENGTH                 = 70.25;
    static final double     TILE_LENGTH                 = 23.5;
    static final double     TILE_WITHOUT_TABS           = 22.75;
    static final double     TILE_1_CENTER               = TILE_WITHOUT_TABS * 0.5;
    static final double     TILE_2_CENTER               = TILE_1_CENTER + TILE_LENGTH;
    static final double     TILE_3_CENTER               = TILE_2_CENTER + TILE_LENGTH;
    static final double     SIDE_WALL_TO_FOUNDATION     = 47.0;
    static final double     BUILD_WALL_TO_FOUNDATION    = 4.0;

    // field elements
    static final double     FOUNDATION_LENGTH           = 37.5;
    static final double     FOUNDATION_WIDTH            = 18.5;
    static final double     STONE_LENGTH                = 8.0;
    static final double     STONE_WIDTH                 = 4.0;
}

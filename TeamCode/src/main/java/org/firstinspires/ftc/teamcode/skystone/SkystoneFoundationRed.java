/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;


@Autonomous(name="RED Foundation+Parking", group="QT")
@Disabled
public class SkystoneFoundationRed extends SkystoneAutoBase {

    static final boolean BLUE = false;

    /* Declare OpMode members. */
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;
    static final double     SLOW_SPEED              = 0.3;
    static final double     BUILD_WALL_TO_ROBOT     = 24.0;
    static final double     MOVE_TOWARDS_FOUNDATION = 24.0;
    static final double     TURN_FOUNDATION_DISTANCE= 64.0;

    protected MecaBot robot = new MecaBot();   // Use a Mecabot's hardware
    protected MecaBotMove nav = new MecaBotMove(this, robot);
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void setOdometryStartingPosition() {
        // Starting postion assumption:
        // Robot is 18x18 square, robot position (x,y) is center of the robot
        // starting position is against the wall (y=0), hence robot center is y = 9 inches
        // Robot (BLUE:right | RED:left) side is exactly on the 1st and 2nd tile line (47 inches from center)
        // Robot back is facing the foundation (+ve Y-axis)
        // The direction of travel from starting position to foundation is backwards or reverse
        // hence robot front face (green wheels) orientation is -ve Y-axis or -90 degrees
        globalPosition.initGlobalPosition(47.0, 9.0, -90.0);

    }

    @Override
    public void runOpMode() {


        aColor = FieldSkystone.AllianceColor.RED;

        // initialize the robot hardware, navigation, IMU, Odometry and Telemetry display
        initializeOpMode();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This OpMode does all the tasks in the autonomous period of 30 seconds
        // TODO: Use the odometry methods to move Foundation not this hack from 1st QT
        moveFoundationToBuildZone();

        // Don't exit, wait for user (driver presses STOP)
        waitForStop();

    }

    /*
     * Starting postion assumption: Robot green wheels touching the perimeter wall folded up
     * Robot (BLUE:right | RED:left) side is exactly on the 1st and 2nd tile line.
     * The direction of travel from starting position to foundation is backwards or reverse
     * Note: Reverse movement is obtained by setting a negative distance (not speed)
     */
    public void moveFoundationToBuildZone() {

        // move away from perimeter wall before moving towards build wall
        nav.encoderMoveForwardBack(-MOVE_TOWARDS_FOUNDATION);

        // move towards the build wall to center robot on foundation long edge
        double robotToFoundationEdge = (FieldSkystone.FOUNDATION_LENGTH - MecaBot.WIDTH) / 2;
        double moveToBuildWall = BUILD_WALL_TO_ROBOT - FieldSkystone.BUILD_WALL_TO_FOUNDATION - robotToFoundationEdge;
        nav.encoderMoveRightLeft(BLUE ? moveToBuildWall : -moveToBuildWall);

        // move backwards towards the foundation now, slowly
        nav.encoderMoveForwardBack((FieldSkystone.SIDE_WALL_TO_FOUNDATION - MecaBot.LENGTH - MOVE_TOWARDS_FOUNDATION) * -1.0, SLOW_SPEED);

        // TODO: check for the contact switch activation here to tell us when robot has reached

        robot.grabFoundation();
        // Allow the servo some time to move
        sleep(1000);

        nav.encoderMoveForwardBack(24);

        double turnDistance = BLUE ? TURN_FOUNDATION_DISTANCE : 130;
        // turn the foundation so that long edge if parallel to the build zone wall
//        nav.encoderTurn(turnDistance, BLUE ? true : false, SLOW_SPEED);
        nav.encoderRotate(40, BLUE ? true : false, SLOW_SPEED);

        // drive the robot in reverse to push the foundation to the build zone wall
        nav.encoderMoveForwardBack(-20);

        robot.releaseFoundation();
        // Allow the servo some time to move
        sleep(1000);

        // Move towards the wall so that we allow space for alliance partner to park
        nav.encoderMoveRightLeft(BLUE ? -12 : -18);

        // now go park under the bridge
        nav.encoderMoveForwardBack(42);
    }

}

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test MecaBotMove")
//@Disabled
public class testMoveMecaBot extends LinearOpMode {

    MecaBot robot = new MecaBot();   // Use Omni-Directional drive system
    MecaBotMove nav = new MecaBotMove(this, robot);

    @Override
    public void runOpMode() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        telemetry.setAutoClear(false);

        fieldConfiguration.initRobotStartX();
        telemetry.addData("Test MecaBotMove Time:", opmodeRunTime.seconds());
        telemetry.addData("robotStartX:", fieldConfiguration.robotStartX);
        telemetry.addData("robotStartY:", fieldConfiguration.robotStartY);
        telemetry.update();

        robot.init(this.hardwareMap);
        waitForStart();
        testDrawBox();

        //nav.moveLeftRight(-12);
        /*
        nav.goPark(0,22, false,false);
        sleep(5000);
        nav.goPark(0,22, true,false);

         */

        //testMoveYX();
        /*
        double inchMove=10;  //2 scan 17 apart; 3 scan
        nav.moveForwardBack(inchMove);
        sleep(2000);
        nav.moveForwardBack(inchMove);
        sleep(2000);

        nav.grabTheStone();
        sleep(2000 );
        nav.releaseTheStone();
*/


        while (opModeIsActive()){

        }

    }

    private void testMoveYX(){
        //draw a box by moving with robot start of heading in positive X direction
        //     move forward,right
        nav.moveYX(10,10,0,0,false);
        //     move back, left
        nav.moveYX(0,0,10,10,false);
        sleep(3000);
        nav.moveYX(-10,0,0,0,false);
        nav.moveYX(0,-10,0,0,false);

    }

    private void testDrawBox(){
        double aInch=10;
        nav.moveForwardBack(aInch);
        nav.moveLeftRight(aInch);
        aInch=-10;
        nav.moveForwardBack(aInch);
        nav.moveLeftRight(aInch);
    }
    public void testColorSensor(){
        ColorSensor cs=robot.groundColorSensor;
        telemetry.addData("Blue Reading=", cs.blue());
        telemetry.addData("Red Reading=", cs.red());
        telemetry.addData("Alpha Reading=", cs.alpha());
        telemetry.update();

    }
}

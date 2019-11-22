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
        robot.init(this.hardwareMap);
        telemetry.setAutoClear(false);

        waitForStart();

        double aInch=10;
        nav.moveForwardBack(aInch);
        nav.moveLeftRight(aInch);
        aInch=-10;
        nav.moveForwardBack(aInch);
        nav.moveLeftRight(aInch);

        nav.grabTheStone();
        sleep(5000 );
        nav.releaseTheStone();

        //testMoveYX();

    }

    private void testMoveYX(){
        //draw a box by moving with robot start of heading in positive X direction
        //     move forward,right
        nav.moveYX(10,10,0,0,true);
        //     move back, left
        nav.moveYX(0,0,10,10,true);

    }
    public void testColorSensor(){
        ColorSensor cs=robot.groundColorSensor;
        telemetry.addData("Blue Reading=", cs.blue());
        telemetry.addData("Red Reading=", cs.red());
        telemetry.addData("Alpha Reading=", cs.alpha());
        telemetry.update();

    }
}

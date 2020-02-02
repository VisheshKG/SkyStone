package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

@Autonomous(name = "Test Color Sensor 4 Stone", group="Test")
@Disabled
public class TestColorSensor extends LinearOpMode {

    MecaBot robot = new MecaBot();
    //MecaBotMove nav = new MecaBotMove(this, robot);
    private LinearOpMode myOpMode=this;       // Access to the OpMode object


    public void runOpMode() {
        robot.init(this.hardwareMap);
        // telemetry.setAutoClear(false);

        isSkyStone();
        waitForStart();
        while (opModeIsActive()){
            if (isSkyStone()){
                //move to get it
            }else {
                sleep(500);
            }
        }

    }

    public boolean isSkyStone(){
        ColorSensor cs = robot.rightColorSensor;

        myOpMode.telemetry.addData("Blue Reading=", cs.blue());
        myOpMode.telemetry.addData("Red Reading=", cs.red());
        myOpMode.telemetry.addData("green   Reading=", cs.green());
        myOpMode.telemetry.addData("Alpha Reading=", cs.alpha());
        float redToblue=((float)(cs.red() - cs.blue()))/(float)cs.blue();
        myOpMode.telemetry.addData("(Red - Blue)/blue=", "%.2f",redToblue);

        myOpMode.telemetry.update();

        if (redToblue < 0.20) {
            myOpMode.telemetry.addData("Skystone Found", redToblue);
            return true;
        }
        return false;
    }


}

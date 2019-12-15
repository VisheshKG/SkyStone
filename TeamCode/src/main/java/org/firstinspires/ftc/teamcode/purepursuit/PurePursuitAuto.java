package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Pure Pursuit Auto", group = "PurePursuit")
public class PurePursuitAuto extends LinearOpMode {

    PureBot robot = new PureBot();

  @Override
  public void runOpMode() {

      robot.init(hardwareMap);

      telemetry.addData(">", "Initialization Complete. Waiting for Start");

      waitForStart();

      while (opModeIsActive()) {
          PureMovement.goToPosition(50, 50, 0.3, 0, 0.2);
          robot.update();

      }
  }


}

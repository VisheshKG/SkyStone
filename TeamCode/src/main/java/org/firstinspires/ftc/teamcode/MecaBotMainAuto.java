package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Disabled
public abstract class MecaBotMainAuto extends LinearOpMode {

    MecaBot robot = new MecaBot();
    private final static int FORWARD_MULTIPLIER = 1;
    private final static int RIGHT_MULTIPLIER = 1;
    private final static int TURN_MULTIPLIER = 1;

    protected void moveForwards(int mm, double speed) {

        //reset the encoders back to 0
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run to the necessary target position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target postion
        //TODO: find the correct constant
        robot.leftBackDrive.setTargetPosition((int) (mm * FORWARD_MULTIPLIER));
        robot.leftFrontDrive.setTargetPosition((int) (mm * FORWARD_MULTIPLIER));
        robot.rightBackDrive.setTargetPosition((int) (mm * FORWARD_MULTIPLIER));
        robot.rightFrontDrive.setTargetPosition((int) (mm * FORWARD_MULTIPLIER));

        // Set the power of the motors to whatever speed is needed
        robot.leftBackDrive.setPower(speed);
        robot.leftFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);

        // Loop until both motors are no longer busy.
        // TODO: find a better way to move than empty loop until reached target position

        while (robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            if (robot.leftBackDrive.getCurrentPosition() > robot.leftBackDrive.getTargetPosition() - 10 && robot.leftBackDrive.getCurrentPosition() < robot.leftBackDrive.getTargetPosition() + 10) {
                if (robot.rightBackDrive.getCurrentPosition() > robot.rightBackDrive.getTargetPosition() - 10 && robot.rightBackDrive.getCurrentPosition() < robot.rightBackDrive.getCurrentPosition() + 10) {
                    break;
                }
            }
        }

        // Stop powering the motors - robot has moved to intended position
        robot.leftBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }

    protected void moveRight(int mm, double speed) {
        //reset the encoders back to 0
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run to the necessary target position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //TODO: Find out how mecanum wheels work with encoders and finish method

        // Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target postion
        //TODO: find the correct constant
        robot.leftBackDrive.setTargetPosition(mm * -RIGHT_MULTIPLIER);
        robot.leftFrontDrive.setTargetPosition(mm * RIGHT_MULTIPLIER);
        robot.rightBackDrive.setTargetPosition(mm * RIGHT_MULTIPLIER);
        robot.rightFrontDrive.setTargetPosition(mm * -RIGHT_MULTIPLIER);

        // Set the power of the motors to whatever speed is needed
        robot.leftBackDrive.setPower(speed);
        robot.leftFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);

        // Loop until both motors are no longer busy.
        // TODO: find a better way to move than empty loop until reached target position

        while (robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            if (robot.leftBackDrive.getCurrentPosition() > robot.leftBackDrive.getTargetPosition() - 10 && robot.leftBackDrive.getCurrentPosition() < robot.leftBackDrive.getTargetPosition() + 10) {
                if (robot.rightBackDrive.getCurrentPosition() > robot.rightBackDrive.getTargetPosition() - 10 && robot.rightBackDrive.getCurrentPosition() < robot.rightBackDrive.getCurrentPosition() + 10) {
                    break;
                }
            }
        }


    }

    protected void turn(int degrees, double speed) {

        //reset the encoders back to 0
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run to the necessary target position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Multiply the distance we require by a determined constant to tell the motors how far to turn/set our target postion
        //TODO: find the correct constant
        robot.leftBackDrive.setTargetPosition((int) (degrees * TURN_MULTIPLIER));
        robot.leftFrontDrive.setTargetPosition((int) (degrees * TURN_MULTIPLIER));
        robot.rightBackDrive.setTargetPosition((int) (degrees * TURN_MULTIPLIER));
        robot.rightFrontDrive.setTargetPosition((int) (degrees * TURN_MULTIPLIER));

        // Set the power of the motors to whatever speed is needed
        robot.leftBackDrive.setPower(speed);
        robot.leftFrontDrive.setPower(speed);
        robot.rightBackDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);


        // Loop until motors are no longer busy
        // TODO: find a better way to move than empty loop until reached target position

        while (robot.leftBackDrive.isBusy() || robot.rightBackDrive.isBusy()) {
            if (robot.leftBackDrive.getCurrentPosition() > robot.leftBackDrive.getTargetPosition() - 10 && robot.leftBackDrive.getCurrentPosition() < robot.leftBackDrive.getTargetPosition() + 10) {
                if (robot.rightBackDrive.getCurrentPosition() > robot.rightBackDrive.getTargetPosition() - 10 && robot.rightBackDrive.getCurrentPosition() < robot.rightBackDrive.getCurrentPosition() + 10) {
                    break;
                }
            }
        }

    }
}

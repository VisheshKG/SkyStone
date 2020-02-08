package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecaBot;
import org.firstinspires.ftc.teamcode.robot.MecaBotMove;

public class SkystoneBotOperator implements Runnable {

    enum ActionMode { IDLE, INTAKE_RUN, INTAKE_STOP, STONE_DELIVER, STONE_LATCH, STONE_RELEASE, LIFT_UP, LIFT_DOWN, LIFTARM_OUT, LIFTARM_IN, FOUNDATION_GRAB, FOUNDATION_RELEASE}

    class OperatorAction {
        // member variables of the Action object
        ActionMode actionMode;
        long startTimer;  // delay to start the action
        long endTimer;    // duration to stop the action AFTER the startTimer
        long reverseActionTimer;  // delay to start reverse action AFTER the startTimer

        public OperatorAction() {
            this(ActionMode.IDLE);
        }
        public OperatorAction(ActionMode mode) {
            this(mode, 0, 0, 0);
        }
        public OperatorAction(ActionMode mode, long start, long end) {
            this(mode, start, end, 0);
        }
        public OperatorAction(ActionMode mode, long start, long end, long reverse) {
            this.actionMode = mode;
            startTimer = start;
            endTimer = end;
            reverseActionTimer = reverse;
        }
    };

    // member variables for state
    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private MecaBot             robot;          // Access to the Robot hardware
    private OperatorAction      action;
    private Thread              myThread;
    private boolean             isRunning = false;

    // This setting for goBilda 5202 series 26.9:1 motor, encoder counts per rotation = 753.2
    //private int[]               liftStops = {MecaBot.LIFT_BOTTOM, 159, 846, 1533, 2220, 2907, 3594, 4281, 4968, 5655, MecaBot.LIFT_TOP};
    // This setting for goBilda 5202 series 50.9:1 motor, encoder counts per rotation = 1425.2
    private int[]               liftStops = {MecaBot.LIFT_BOTTOM, 300, 1600, 2900, 4200, 5500, 6800, 8100, 9400, 10700, MecaBot.LIFT_TOP};

    /* Constructor */
    public SkystoneBotOperator(LinearOpMode opMode, MecaBot aRobot) {
        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        robot = aRobot;
        action = null; // no action is waiting yet
    }

    public void start() {

        isRunning = true;
        myThread = new Thread(this);
        myThread.start();
    }

    public void stop() {
        isRunning = false;
        myThread = null;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            try {
                actionUpdate();
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * actionPerform: requests the action to be performed by the Skystone bot operator
     * Note: The action requestor and the action performer (this class) are in separate threads.
     * However, in current design only one action is expected at a time. The startTimer delay is
     * expected to be zero or small 1 second, therefore the action should be picked up or execution
     * before any next action request arrives.
     * This method will not do much unless the thread has been started by calling the start()
     *
     * @param act    specifies the type of action to be performed
     */
    public void actionPerform(OperatorAction act) {
        if (action != null) {
            myOpMode.telemetry.addData("DEBUG", "Action waiting while NEW action arrived");
        }
        // There is no queueing mechanism yet, we just override the previous waiting action, if any
        action = act;
    }

    private void actionUpdate() {
        if (action == null) {
            return;
        }
        myOpMode.telemetry.addLine("OPER").addData("Action", action.actionMode)
                .addData("D", action.startTimer).addData("E", action.endTimer)
                .addData("R", action.reverseActionTimer);

        // We will sleep for startTimer duration, only 1 action at a time expected
        if (action.startTimer > 0) {
            myOpMode.sleep(action.startTimer);
        }

        switch (action.actionMode) {
            case IDLE:
                break;
            case INTAKE_RUN:
                robot.runIntake(MecaBotMove.DRIVE_SPEED_DEFAULT);
                if (action.endTimer > 0) {
                    // We will sleep for endTimer duration, only 1 action at a time expected
                    myOpMode.sleep(action.endTimer);
                    robot.stopIntake();
                }
                break;
            case INTAKE_STOP:
                robot.stopIntake();
                break;
            case STONE_DELIVER:
                robot.grabStoneWithClaw();
                moveLiftToPosition(liftStops[1]*2);
                moveLiftArmOutside();
                robot.releaseStoneWithClaw();
                moveLiftToPosition(liftStops[2]); // need to go up 2 levels to clear stone for arm withdrawal
                moveLiftArmInside();
                moveLiftToPosition(MecaBot.LIFT_BOTTOM);
                break;
            case STONE_LATCH:
                robot.grabStoneWithClaw();
                break;
            case STONE_RELEASE:
                robot.releaseStoneWithClaw();
                break;
            case LIFT_UP:
                moveLiftUp();
                if (action.reverseActionTimer > 0) {
                    // We will sleep for endTimer duration, only 1 action at a time expected
                    myOpMode.sleep(action.reverseActionTimer);
                    moveLiftDown();
                }
                break;
            case LIFT_DOWN:
                moveLiftDown();
                break;
            case LIFTARM_OUT:
                this.moveLiftArmOutside();
                if (action.reverseActionTimer > 0) {
                    // We will sleep for endTimer duration, only 1 action at a time expected
                    myOpMode.sleep(action.reverseActionTimer);
                    moveLiftArmInside();
                }
                break;
            case LIFTARM_IN:
                moveLiftArmInside();
                break;
            case FOUNDATION_GRAB:
                robot.grabFoundation();
                if (action.reverseActionTimer > 0) {
                    // We will sleep for endTimer duration, only 1 action at a time expected
                    myOpMode.sleep(action.reverseActionTimer);
                    robot.releaseFoundation();
                }
                break;
            case FOUNDATION_RELEASE:
                robot.releaseFoundation();
                break;
            default:
                // nothing to do
        }
        action = null;
    }

    public void moveLiftToPosition(int position) {

        robot.moveLift(position);
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && robot.liftMotor.isBusy() && runTime.seconds() < 1.5) {
            myOpMode.telemetry.addData("LIFT ", "auto moving to %d", position);
            myOpMode.telemetry.update();
        }
        robot.stopLift();
    }

    public void moveLiftDown() {
        int pos = robot.liftMotor.getCurrentPosition();
        int target = pos;
        for (int i=0; i<liftStops.length - 1; i++) {
            // whatever lift stop is higher or equal to current position, go down one stop below
            if (liftStops[i+1] - pos > -100) {
                target = liftStops[i];
                break;
            }
        }
        moveLiftToPosition(target);
    }

    public void moveLiftUp() {
        int pos = robot.liftMotor.getCurrentPosition();
        int target = pos;
        // whatever lift stop is higher than current position, go up to that stop
        for (int i=0; i<liftStops.length; i++) {
            if (liftStops[i] - pos > 100) {
                target = liftStops[i];
                break;
            }
        }
        moveLiftToPosition(target);
    }

    public void moveLiftArmToPosition(int position) {

        robot.moveLiftArm(position);
        ElapsedTime runTime = new ElapsedTime();
        while (myOpMode.opModeIsActive() && robot.liftArmMotor.isBusy() && runTime.seconds() < 1.5) {
            myOpMode.telemetry.addData("ARM ", "auto moving to %d", position);
            myOpMode.telemetry.update();
        }
        robot.liftArmMotor.setPower(MecaBotMove.DRIVE_SPEED_BRAKE);
        robot.liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLiftArmInside() {
        moveLiftArmToPosition(MecaBot.ARM_INSIDE);
    }

    public void moveLiftArmOutside() {
        moveLiftArmToPosition(MecaBot.ARM_OUTSIDE);
    }

}

package org.firstinspires.ftc.teamcode.purepursuit;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.purepursuit.PureBot.*;
import static org.firstinspires.ftc.teamcode.purepursuit.MovementVars.*;
import static org.firstinspires.ftc.teamcode.purepursuit.MathFunctions.angleWrap;

public class PureMovement {

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {


        // TODO: need the starting position of the robot
        double distanceToTarget = Math.hypot(x = worldXPosition, y - worldYPosition);

        // The angle to get to correct vector (from 0)
        // TODO: need the starting position of the robot
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        // The relative angle from the robot to the point
        // TODO: need the current angle of the robot
        double relativeAngletoPoint = angleWrap(absoluteAngleToTarget - worldAngle);


        double relativeXtoPoint = Math.cos(relativeAngletoPoint) * distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngletoPoint) * distanceToTarget;


        // Normalize the power so that the robot is moving at a consistant rate
        // No matter how big the vector is, power is from 0 to 1, but same ratio is x/y to total
        double movementXPower = relativeXtoPoint /  Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint);
        double movementYPower = relativeYtoPoint / Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint);

        // Set the Power
        // TODO: incorporate robot + setting power (see robot.java in tutorial git)
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        // Creates an angle for the robot to turn to starting at relativeAngletoPoint
        double relativeTurnAngle = relativeAngletoPoint - 180 + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / 30, -1, 1) * turnSpeed;

        if (distanceToTarget < 10) {
            movement_turn = 0;
        }

    }

}

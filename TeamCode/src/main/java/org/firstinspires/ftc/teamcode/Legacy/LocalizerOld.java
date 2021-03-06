package org.firstinspires.ftc.teamcode.Legacy;

import org.firstinspires.ftc.teamcode.Odometer;
import org.firstinspires.ftc.teamcode.Point;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.MathHelper.wrapAngle;

//Class that represents the whole odometry system and contains three odometer objects, one for each wheel
public class LocalizerOld {

    public OdometerOld left, right, center;

    public Point robotPosition;
    public double robotAngle;

    public double deltaX;
    public double deltaY;
    public double deltaAngle;

    private double lastResetAngle; //Using for absolute turn calculations

    public LocalizerOld(OdometerOld left, OdometerOld right, OdometerOld center) {
        this.left = left;
        this.right = right;
        this.center = center;

        robotPosition = new Point();

        robotPosition.x = 0;
        robotPosition.y = 0;
        robotAngle = 0;

        lastResetAngle = 0;
    }

    public void update() {
        left.update();
        right.update();
        center.update();

        deltaAngle = ((right.delta - left.delta) / 2) / ((right.radius + left.radius) / 2); //Average of the two values divided by average of the radius (with is the same anyway); counterclockwise rotation is positive

        double absoluteAngle = wrapAngle(((right.totalDelta - left.totalDelta)/2 / ((right.radius + left.radius) / 2)) + lastResetAngle);

        double xError = (deltaAngle * center.radius); // If we turn counterclockwise (positive), the center odometer will track right (positive)

        deltaY = (right.delta + left.delta) / 2;
        deltaX = center.delta - xError;


        if (Math.abs(deltaAngle) > 0) { //These are the arc calculations that avoid just segmenting our motion
            double movementRadius = (right.delta + left.delta) / (2*deltaAngle);
            double strafeRadius = (deltaX / deltaAngle);

            deltaY = (movementRadius * Math.sin(deltaAngle)) - (strafeRadius * (1-Math.cos(deltaAngle)));
            deltaX = (movementRadius * (1-Math.cos(deltaAngle))) + (strafeRadius * Math.sin(deltaAngle));

        }

        robotPosition.x += (Math.cos(robotAngle) * deltaY) + (Math.sin(robotAngle) * deltaX);
        robotPosition.y += (Math.sin(robotAngle) * deltaY) - (Math.cos(robotAngle) * deltaX);
        robotAngle = absoluteAngle;
    }

    //Used to just set the position, such as at the beginning of an opmode
    public void setPosition(Point robotPosition, double robotAngle) {
        this.robotPosition = robotPosition;
        this.robotAngle = robotAngle;

        this.lastResetAngle = robotAngle;

        left.lastResetValue = left.currentValue;
        right.lastResetValue = right.currentValue;
        center.lastResetValue = center.currentValue;
    }

}

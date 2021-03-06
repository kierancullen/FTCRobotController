package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MathHelper.wrapAngle;

//Class that represents the whole odometry system and contains three odometer objects, one for each wheel
public class Localizer {

    public Odometer left, right, center;

    public Point robotPosition;
    public double robotAngle;

    public double deltaX;
    public double deltaY;
    public double deltaAngle;

    private double lastResetAngle; //Using for absolute turn calculations

    public double verticalScalingFactor = 0.012972;
    public double horizontalScalingFactor = 0.01272;
    public double turnScalingFactor = 0.0003164943;
    public double correctionScalingFactor = 12.45;


    public Localizer(Odometer left, Odometer right, Odometer center) {
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

        double rightDeltaScale = right.deltaRaw * verticalScalingFactor;
        double leftDeltaScale = left.deltaRaw * verticalScalingFactor;
        double centerDeltaScale = center.deltaRaw * horizontalScalingFactor;

        deltaAngle = (right.deltaRaw - left.deltaRaw) * turnScalingFactor;

        double absoluteAngle = wrapAngle((right.totalDeltaRaw - left.totalDeltaRaw) * turnScalingFactor + lastResetAngle);

        double xError = (deltaAngle * correctionScalingFactor); // If we turn counterclockwise (positive), the center odometer will track right (positive)

        deltaY = (rightDeltaScale + leftDeltaScale) / 2;
        deltaX = centerDeltaScale - xError;


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

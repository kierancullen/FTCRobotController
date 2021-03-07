package org.firstinspires.ftc.teamcode.GoToPosition;

import org.firstinspires.ftc.teamcode.Point;

public class GTPOpModeVariables {

    //Here is where all the points will be defined for the GoToPosition OpMode
        public Point location;
        public double staticAngle; //The angle that the robot should be at once we get to the point

        public double driveAngle; //The angle between the robot and the point while driving
        public double goToSpeed; //Speed to drive to the point at
        public double goToSpeedTurn; //Speed to turn at while driving and orienting

        public double slowDownDistance; //Distance to the point within which to start slowing down
        public double slowDownAngle; //Angle to the target within which to start slowing down turning

        boolean goThroughPoint; //Whether or not the point is part of a larger set and the robot should stop or continue through it

        public void GTP (Point location, double staticAngle, double driveAngle, double goToSpeed, double goToSpeedTurn, double slowDownDistance, double slowDownAngle, boolean goThroughPoint) {
            this.location = location;
            this.staticAngle = staticAngle;
            this.driveAngle = driveAngle;
            this.goToSpeed = goToSpeed;
            this.goToSpeedTurn = goToSpeedTurn;
            this.slowDownDistance = slowDownDistance;
            this.slowDownAngle = slowDownAngle;
            this.goThroughPoint = goThroughPoint;
        }
}

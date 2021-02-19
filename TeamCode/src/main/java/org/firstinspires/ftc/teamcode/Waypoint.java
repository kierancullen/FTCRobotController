package org.firstinspires.ftc.teamcode;

public class Waypoint {
    public Point location;
    public double staticAngle;

    public double driveAngle; //The angle between the robot and the point while driving)
    public double goToSpeed; //Speed to drive to the point at
    public double goToSpeedTurn; //Speed to turn at while driving and orienting

    public double slowDownDistance;
    public double slowDownAngle;

    public Waypoint(Point location, double staticAngle, double driveAngle, double goToSpeed, double goToSpeedTurn, double slowDownDistance, double slowDownAngle) {
        this.location = location;
        this.staticAngle = staticAngle;
        this.driveAngle = driveAngle;
        this.goToSpeed = goToSpeed;
        this.goToSpeedTurn = goToSpeedTurn;
        this.slowDownDistance = slowDownDistance;
        this.slowDownAngle = slowDownAngle;
    }
}

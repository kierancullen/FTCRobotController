package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

//Really just an add-on to the localizer class that tracks the robot speed and estimates things like slip distance
//This functionality could've been implemented in the localizer class probably

public class SpeedTracker {

    private Localizer localizer;

    private long lastUpdateStartTime = 0;

    public Point currentSpeed;

    public double currentAngularVelocity = 0;

    private static int timeBetweenUpdates = 25;
    private static double ySlipDistance = 0.14; //for 1 centimeter per second robot speed
    private static double xSlipDistance = 0.153;
    public static double turnSlipAmount = 0.09; //for 1 radian per second

    public SpeedTracker(Localizer localizer) {
        this.localizer = localizer;
        currentSpeed = new Point (0,0);
    }

    public void update() {
        long currentTime = SystemClock.uptimeMillis();

        //If we haven't actually moved on this iteration of the loop, just return
        if (Math.abs(localizer.deltaX) < 0.0001 && Math.abs(localizer.deltaY) < 0.0001 && Math.abs(localizer.deltaAngle) < 0.0001) return;

        //Make sure we're not updating too fast
        if (currentTime - lastUpdateStartTime > timeBetweenUpdates) {
            double elapsedTimeSec = (double)(currentTime - lastUpdateStartTime) / 1000.0;
            double speedY = localizer.deltaY / elapsedTimeSec; //Real speed in cm/s
            double speedX = localizer.deltaX / elapsedTimeSec; //Real speed in cm/s

            if (speedX < 500 && speedY < 500) { //Sanity check on the speed to prevent weird spikes
                currentSpeed.x = speedX;
                currentSpeed.y = speedY;
            }

            currentAngularVelocity = MathHelper.wrapAngle(localizer.deltaAngle) / elapsedTimeSec;
            lastUpdateStartTime = currentTime;
        }
    }

    //Calculates how far the robot would slip if we were to stop moving at the current instant
    //I'm using Point as a distance vector here really
    public Point getCurrentSlipDistance() {
        double slipDistanceY = currentSpeed.y * ySlipDistance;
        double slipDistanceX = currentSpeed.x * xSlipDistance;
        return new Point (slipDistanceX, slipDistanceY);
    }

    //Calcuates how far the robot would slip rotationally if we were to stop moving at the current instant
    public double getCurrentSlipAngle() {
        return currentAngularVelocity * turnSlipAmount;
    }

}


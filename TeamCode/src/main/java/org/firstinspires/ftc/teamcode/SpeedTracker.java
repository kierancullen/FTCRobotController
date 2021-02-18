package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

public class SpeedTracker {

    private Localizer localizer;

    private long lastUpdateStartTime = 0;

    private double currentSpeedX = 0.0;
    private double currentSpeedY = 0.0;

    public double currentAngularVelocity = 0;

    private static int timeBetweenUpdates = 25;
    private static double ySlipDistance = 0.14; //for 1 centimeter per second robot speed
    private static double xSlipDistance = 0.153;
    public static double turnSlipAmount = 0.09; //for 1 radian per second

    public SpeedTracker(Localizer localizer) {
        this.localizer = localizer;
    }

    public void update() {
        long currentTime = SystemClock.uptimeMillis();

        if (Math.abs(localizer.deltaX) < 0.0001 && Math.abs(localizer.deltaY) < 0.0001 && Math.abs(localizer.deltaAngle) < 0.0001) return;

        if (currentTime - lastUpdateStartTime > timeBetweenUpdates) {
            double elapsedTimeSec = (double)(currentTime - lastUpdateStartTime) / 1000.0;
            double speedY = localizer.deltaY / elapsedTimeSec;
            double speedX = localizer.deltaX / elapsedTimeSec;

            if (speedX < 500 && speedY < 500) {
                currentSpeedX = speedX;
                currentSpeedY = speedY;
            }

            currentAngularVelocity = MathHelper.wrapAngle(localizer.deltaAngle) / elapsedTimeSec;
            lastUpdateStartTime = currentTime;
        }
    }

    public Point getCurrentSlipDistance() {
        double slipDistanceY = currentSpeedY * ySlipDistance;
        double slipDistanceX = currentSpeedX * xSlipDistance;
        return new Point (slipDistanceX, slipDistanceY);
    }

    public double getCurrentSlipAngle() {
        return currentAngularVelocity * turnSlipAmount;
    }

}


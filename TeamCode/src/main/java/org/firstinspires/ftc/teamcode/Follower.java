package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

//A class that contains the methods for going to points
//A Follower object has access to the localizer (odometry), SpeedTracker, and drivetrain, and can set the drivetrain powers based on the robot location

public class Follower {

    Localizer localizer;
    SpeedTracker tracker;
    Drivetrain drivetrain;

    public state movementXState;
    public state movementYState;
    public state turningState;

    private static final double smallAdjustSpeed = 0.135;

    //Mini state machine for motion profiling
    public enum state {
        zooming,
        decelerating,
        adjusting;

        private static state[] states = values();
        public state next() {
            return states[(this.ordinal() + 1) % states.length];
        }
    }


    public Follower (Localizer localizer, Drivetrain drivetrain, SpeedTracker tracker) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        this.tracker = tracker;
    }


    //This has to be called before each new move
    public void initialize () {
        movementXState = state.zooming;
        movementYState = state.zooming;
        turningState = state.zooming;
    }

    //Not using this
    public void goTo(Point target, double angle, double movementSpeed, double turnSpeed) {
        double distanceToPoint = Math.sqrt(Math.pow(target.x-localizer.robotPosition.x, 2) + Math.pow(target.y-localizer.robotPosition.y, 2));

        double angleToPoint = Math.atan2(target.y - localizer.robotPosition.y, target.x - localizer.robotPosition.x);
        double deltaAngleToPoint = MathHelper.wrapAngle(angleToPoint-(localizer.robotAngle-Math.toRadians(90))); //check this

        double relativeX = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relativeY = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relativeXAbs = Math.abs(relativeX);
        double relativeYAbs = Math.abs(relativeY);

        double powerX = (relativeX / (relativeYAbs + relativeXAbs)) * movementSpeed;
        double powerY = (relativeY / (relativeYAbs + relativeXAbs)) * movementSpeed;

        if (movementYState == state.zooming) {
            if(relativeYAbs < Math.abs(tracker.getCurrentSlipDistance().y * 2) || relativeYAbs < 3) {
                movementYState = movementYState.next();
            }
        }

        if (movementYState == state.decelerating) {
            powerY = 0;
            if (Math.abs(tracker.currentSpeed.y) < 3) {
                movementYState = movementYState.next();
            }
        }

        if (movementYState == state.adjusting) {
            powerY = Range.clip(((relativeY/8.0) * 0.15), -0.15, 0.15);
        }

        if (movementXState == state.zooming) {
            if(relativeXAbs < Math.abs(tracker.getCurrentSlipDistance().y * 1.2) || relativeXAbs < 3) {
                movementXState = movementXState.next();
            }
        }

        if (movementXState == state.decelerating) {
            powerX = 0;
            if (Math.abs(tracker.currentSpeed.y) < 3) {
                movementXState = movementXState.next();
            }
        }

        if (movementXState == state.adjusting) {
            powerX = Range.clip(((relativeX/2.5) * smallAdjustSpeed), -smallAdjustSpeed, smallAdjustSpeed);
        }

        double turnDistance = MathHelper.wrapAngle(angle - localizer.robotAngle);
        double powerTurn = 0;

        if (turningState == state.zooming) {
            powerTurn = turnDistance > 0 ? turnSpeed: -turnSpeed;
            if (Math.abs(turnDistance) < Math.abs(tracker.getCurrentSlipAngle() * 1.2) || Math.abs(turnDistance) < Math.toRadians(3.0)) {
                turningState = turningState.next();
            }
        }

        if (turningState == state.decelerating) {
            if (Math.toDegrees(Math.abs(tracker.currentAngularVelocity)) < 60) {
                turningState = turningState.next();
            }
        }

        if (turningState == state.adjusting) {
            powerTurn = (turnDistance/Math.toRadians(10) * smallAdjustSpeed);
            powerTurn = Range.clip(powerTurn, -smallAdjustSpeed, smallAdjustSpeed);
        }

        drivetrain.turnVelocity = powerTurn;
        drivetrain.translateVelocity.x = powerX;
        drivetrain.translateVelocity.y = powerY;

        minimizeComponents();

    }

    //Not using this
    public void goToSimple(Point target, double angle, double movementSpeed, double turnSpeed, double slowDownTurnRadians, double slowDownMovementFromTurnError, boolean moveThrough) {

        double currentSlipY = (tracker.getCurrentSlipDistance().y * Math.sin(localizer.robotAngle) +
                                tracker.getCurrentSlipDistance().x * Math.cos(localizer.robotAngle));
        double currentSlipX = (tracker.getCurrentSlipDistance().y * Math.cos(localizer.robotAngle) +
                tracker.getCurrentSlipDistance().x * Math.sin(localizer.robotAngle));

        Point targetAdjusted = new Point (target.x - currentSlipX, target.y - currentSlipY);

        double distanceToPoint = Math.sqrt(Math.pow(targetAdjusted.x-localizer.robotPosition.x, 2) + Math.pow(targetAdjusted.y-localizer.robotPosition.y, 2));

        double angleToPoint = Math.atan2(targetAdjusted.y - localizer.robotPosition.y, targetAdjusted.x - localizer.robotPosition.x);
        double deltaAngleToPoint = MathHelper.wrapAngle(angleToPoint-(localizer.robotAngle-Math.toRadians(90))); //check this

        double relativeX = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relativeY = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relativeXAbs = Math.abs(relativeX);
        double relativeYAbs = Math.abs(relativeY);

        double powerX = (relativeX / (relativeYAbs + relativeXAbs));
        double powerY = (relativeY / (relativeYAbs + relativeXAbs));

        if (!moveThrough) {
            powerX *= relativeXAbs / 40.0;
            powerY *= relativeYAbs / 40.0;
        }

        powerX = Range.clip(powerX, -movementSpeed, movementSpeed);
        powerY = Range.clip(powerY, - movementSpeed, movementSpeed);

        double absoluteHeading = angle+Math.atan2(target.y - localizer.robotPosition.y, target.x - localizer.robotPosition.x);
        double relativeHeading = MathHelper.wrapAngle(absoluteHeading-localizer.robotAngle);

        double relativeHeadingAdjusted = MathHelper.wrapAngle(relativeHeading-tracker.getCurrentSlipAngle());

        double decelerationDistance = Math.toRadians(40);

        double powerTurn = (relativeHeadingAdjusted/decelerationDistance) * turnSpeed;

        powerTurn = Range.clip(powerTurn, -turnSpeed, turnSpeed);

        if (distanceToPoint < 30) {
            powerTurn = 0; //Prevents orbiting
        }

        drivetrain.turnVelocity = powerTurn;
        drivetrain.translateVelocity.x = powerX;
        drivetrain.translateVelocity.y = powerY;

        minimizeComponents();

        //Smoothing
        drivetrain.translateVelocity.x *= Range.clip((relativeXAbs/6.0), 0, 1);
        drivetrain.translateVelocity.y *= Range.clip((relativeYAbs/6.0), 0, 1);
        drivetrain.turnVelocity *= Range.clip(Math.abs(relativeHeading)/Math.toRadians(2), 0, 1);

        //Slowing down if our angle isn't correct and we're getting close to the point
        double turnErrorScale = Range.clip(1.0-Math.abs(relativeHeading/slowDownTurnRadians), 1.0-slowDownMovementFromTurnError, 1);
        if (Math.abs(drivetrain.turnVelocity) < 0.0001) {
            turnErrorScale = 1.0;
        }

        drivetrain.translateVelocity.x *= turnErrorScale;
        drivetrain.translateVelocity.y *= turnErrorScale;

    }

    //Latest method with all the new functionality
    public void goToWaypoint(Waypoint target, boolean stable) {

        //Determine how the robot would slip if we cut the powers, based on our current speed
        double currentSlipY = (tracker.getCurrentSlipDistance().y * Math.sin(localizer.robotAngle) +
                tracker.getCurrentSlipDistance().x * Math.cos(localizer.robotAngle));
        double currentSlipX = (tracker.getCurrentSlipDistance().y * Math.cos(localizer.robotAngle) +
                tracker.getCurrentSlipDistance().x * Math.sin(localizer.robotAngle));

        //Adjust the target based on how we would slip
        Point targetAdjusted = new Point (target.location.x - currentSlipX, target.location.y - currentSlipY);

        double distanceToPoint = Math.sqrt(Math.pow(targetAdjusted.x-localizer.robotPosition.x, 2) + Math.pow(targetAdjusted.y-localizer.robotPosition.y, 2));

        double angleToPoint = Math.atan2(targetAdjusted.y - localizer.robotPosition.y, targetAdjusted.x - localizer.robotPosition.x);
        double deltaAngleToPoint = MathHelper.wrapAngle(angleToPoint-(localizer.robotAngle-Math.toRadians(90))); //check this

        double relativeX = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relativeY = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relativeXAbs = Math.abs(relativeX);
        double relativeYAbs = Math.abs(relativeY);

        double powerX = (relativeX / (relativeYAbs + relativeXAbs));
        double powerY = (relativeY / (relativeYAbs + relativeXAbs));

        //State machines for translation
        if (movementYState == state.zooming) {
            powerY = Range.clip(powerY, -target.goToSpeed, target.goToSpeed); //We're going fast, so just clip the power to be within the maximum
            if(relativeYAbs < target.slowDownDistance && stable) {
                movementYState = state.decelerating; //Go to the next state if we're within the slowDownDistance
            }
        }

        if (movementYState == state.decelerating) {
            powerY *= relativeYAbs / target.slowDownDistance; //Gradually slow down over the slowDownDistance
            powerY = Range.clip(powerY, -target.goToSpeed, target.goToSpeed);
            powerY = minPower(powerY, yMin*2); //Don't want to go under the minimum power when we're too far away
            if (relativeYAbs < 5) { //Go to the next state if we're within 5cm
                movementYState = movementYState.adjusting;
            }
        }

        if (movementYState == state.adjusting) {
            powerY = Range.clip(powerY, -yMin * 2, yMin * 2); //Use very small power to make fine adjustments
            powerY *= Range.clip((relativeYAbs/2.0), 0, 1);
        }

        //All of these are duplicates of the ones for the Y direction
        if (movementXState == state.zooming) {
            powerX = Range.clip(powerX, -target.goToSpeed, target.goToSpeed);
            if(relativeXAbs < target.slowDownDistance && stable) {
                movementXState = state.decelerating;
            }
        }

        if (movementXState == state.decelerating) {
            powerX *= relativeXAbs / target.slowDownDistance;
            powerX = Range.clip(powerX, -target.goToSpeed, target.goToSpeed);
            powerX = minPower(powerX, xMin*2); //Don't want to go under the minimum power when we're too far away
            if (relativeXAbs < 5) {
                movementXState = movementYState.adjusting;
            }
        }

        if (movementXState == state.adjusting) {
            powerX = Range.clip(powerX, -xMin * 2, xMin * 2);
            powerX *= Range.clip((relativeXAbs/2.0), 0, 1);
        }

        //Heading calculations:
        double absoluteHeading = target.driveAngle + Math.atan2(target.location.y - localizer.robotPosition.y, target.location.x - localizer.robotPosition.x);
        double relativeHeading = MathHelper.wrapAngle(absoluteHeading-localizer.robotAngle);

        double relativeHeadingAdjusted = MathHelper.wrapAngle(relativeHeading-tracker.getCurrentSlipAngle());

        double staticTurnDistance = MathHelper.wrapAngle(target.staticAngle - localizer.robotAngle);

        double powerTurn = 0;

        if (turningState == state.zooming) { //In this state, we're turning so that we drive towards the target point at driveAngle
            powerTurn = (relativeHeadingAdjusted/target.slowDownAngle) * target.goToSpeedTurn;
            powerTurn = Range.clip(powerTurn, -target.goToSpeedTurn, target.goToSpeedTurn);
            if (relativeXAbs < target.slowDownDistance && relativeYAbs < target.slowDownDistance && stable) { //Once we get close, we go to the next state
                turningState = state.adjusting;
            }
        }

        if (turningState == state.adjusting) { //In this state, we're turning so that the robot's absolute angle is staticAngle
            powerTurn = (staticTurnDistance/target.slowDownAngle) * target.goToSpeedTurn;
            powerTurn = Range.clip(powerTurn, -target.goToSpeedTurn, target.goToSpeedTurn);
            powerTurn = minPower(powerTurn, turnMin);
            powerTurn *= Range.clip(Math.abs(staticTurnDistance)/Math.toRadians(1), 0, 1);

        }

        if(relativeXAbs == 0 && relativeYAbs == 0) { //Fixes a case where we divide by 0 if we're extactly on top of the taet point
            powerX = 0;
            powerY = 0;
        }

        //Set the drivetrain velocities
        drivetrain.turnVelocity = powerTurn;
        drivetrain.translateVelocity.x = powerX;
        drivetrain.translateVelocity.y = powerY;
    }

    //These are theoretically the minimum motor powers that produce any movement in each direction
    double xMin = 0.11;
    double yMin = 0.091;
    double turnMin = 0.1;

    //Not actually using this anymore
    private void minimizeComponents() {
        if(Math.abs(drivetrain.translateVelocity.x) > Math.abs(drivetrain.translateVelocity.y)) {
            if (Math.abs(drivetrain.translateVelocity.x) > Math.abs(drivetrain.turnVelocity)) {
                drivetrain.translateVelocity.x = minPower(drivetrain.translateVelocity.x, xMin);
            }
            else {
                drivetrain.turnVelocity = minPower(drivetrain.turnVelocity, turnMin);
            }
        }
        else {
            if (Math.abs(drivetrain.translateVelocity.y) > Math.abs(drivetrain.turnVelocity)) {
                drivetrain.translateVelocity.y = minPower(drivetrain.translateVelocity.y, yMin);
            }
            else {
                drivetrain.turnVelocity = minPower(drivetrain.turnVelocity, turnMin);
            }
        }
    }

    //If min is the minimum power that will produce movement, this method returns either value or min, whiever is higher (disregarding sign).
    public double minPower(double value, double min) {
        if (value >= 0 && value <= min) {
            return min;
        }
        if (value <0 && value >-min) {
            return -min;
        }
        return value;
    }

    //Helper method that gets our current angle to a target point (useful for aiming at stuff)
    public double angleTo(Point target) {
        return Math.atan2(target.y - localizer.robotPosition.y, target.x - localizer.robotPosition.x);
    }

    //Helper method that gets our current distance to a target point
    public double distanceTo(Point target) {
        return Math.sqrt(Math.pow(target.x-localizer.robotPosition.x, 2) + Math.pow(target.y-localizer.robotPosition.y, 2));
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Follower {

    Localizer localizer;
    SpeedTracker tracker;
    Drivetrain drivetrain;

    private state movementXState;
    private state movementYState;
    private state turningState;

    private static final double smallAdjustSpeed = 0.135;

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


    public void initialize () {
        movementXState = state.zooming;
        movementYState = state.zooming;
        turningState = state.zooming;
    }

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

    public void goToWaypoint(Waypoint target, boolean stable) { //new method with all the latest functionality

        double currentSlipY = (tracker.getCurrentSlipDistance().y * Math.sin(localizer.robotAngle) +
                tracker.getCurrentSlipDistance().x * Math.cos(localizer.robotAngle));
        double currentSlipX = (tracker.getCurrentSlipDistance().y * Math.cos(localizer.robotAngle) +
                tracker.getCurrentSlipDistance().x * Math.sin(localizer.robotAngle));

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

        if (movementYState == state.zooming) {
            powerY = Range.clip(powerY, -target.goToSpeed, target.goToSpeed);
            if(relativeYAbs < target.slowDownDistance && stable) {
                movementYState = state.decelerating;
            }
        }

        if (movementYState == state.decelerating) {
            powerY *= relativeYAbs / target.slowDownDistance;
            powerY = Range.clip(powerY, -target.goToSpeed, target.goToSpeed);
            if (Math.abs(tracker.currentSpeed.y) < 3) {
                movementYState = movementYState.adjusting;
            }
        }

        if (movementYState == state.adjusting) {
            powerY *= relativeYAbs / target.slowDownDistance;
            powerY = Range.clip(powerY, -target.goToSpeed, target.goToSpeed);
            powerY *= Range.clip((relativeYAbs/6.0), 0, 1); //Check this
        }

        if (movementXState == state.zooming) {
            powerX = Range.clip(powerX, -target.goToSpeed, target.goToSpeed);
            if(relativeXAbs < target.slowDownDistance && stable) {
                movementXState = state.decelerating;
            }
        }

        if (movementXState == state.decelerating) {
            powerX *= relativeXAbs / target.slowDownDistance;
            powerX = Range.clip(powerX, -target.goToSpeed, target.goToSpeed);
            if (Math.abs(tracker.currentSpeed.x) < 3) {
                movementXState = movementYState.adjusting;
            }
        }

        if (movementXState == state.adjusting) {
            powerX *= relativeXAbs / target.slowDownDistance;
            powerX = Range.clip(powerX, -target.goToSpeed, target.goToSpeed);
            powerX *= Range.clip((relativeXAbs/6.0), 0, 1); //Check this
        }

        double absoluteHeading = target.driveAngle + Math.atan2(target.location.y - localizer.robotPosition.y, target.location.x - localizer.robotPosition.x);
        double relativeHeading = MathHelper.wrapAngle(absoluteHeading-localizer.robotAngle);

        double relativeHeadingAdjusted = MathHelper.wrapAngle(relativeHeading-tracker.getCurrentSlipAngle());

        double staticTurnDistance = MathHelper.wrapAngle(target.staticAngle - localizer.robotAngle);

        double powerTurn = 0;

        if (turningState == state.zooming) {
            powerTurn = (relativeHeadingAdjusted/target.slowDownAngle);
            powerTurn = Range.clip(powerTurn, -target.goToSpeedTurn, target.goToSpeedTurn);
            if (relativeXAbs < target.slowDownDistance && relativeYAbs < target.slowDownDistance && stable) {
                turningState = state.adjusting;
            }
        }

        if (turningState == state.adjusting) {
            powerTurn = (staticTurnDistance/target.slowDownAngle);
            powerTurn = Range.clip(powerTurn, -target.goToSpeedTurn, target.goToSpeedTurn);
            powerTurn *= Range.clip(Math.abs(staticTurnDistance)/Math.toRadians(2), 0, 1);

        }
        drivetrain.turnVelocity = powerTurn;
        drivetrain.translateVelocity.x = powerX;
        drivetrain.translateVelocity.y = powerY;
    }

    double xMin = 0.11;
    double yMin = 0.091;
    double turnMin = 0.1;
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

    public double minPower(double value, double min) {
        if (value >= 0 && value <= min) {
            return min;
        }
        if (value <0 && value >-min) {
            return -min;
        }
        return value;
    }


}

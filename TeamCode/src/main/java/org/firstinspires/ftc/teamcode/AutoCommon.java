package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoCommon")
public class AutoCommon extends BaseOpmode {


    enum state {
        starting,
        driveout1, driveout2,
        pause1, powershot1,
        strafe1, pause2, powershot2,
        strafe2, pause3, powershot3,
        ringthings,
        aim, launchextras,
        ringthings2,
        aim2, launchextras2,
        drivetodropzone, dropwobble,
        driveback, grabwobble,
        drivetodropzone2, dropwobble2,
        park
    }

    final double normalRPM = 5000;
    final double powershotRPM = 3600;

    Point robotStartPosition = new Point (121.92,21.955);
    Point noahOrigin = new Point (121.92, 21.955); //The point that I think all of Noah's points are relative to

    Waypoint driveout1 = new Waypoint(transform(10, 100, noahOrigin), Math.toRadians(90), Math.toRadians(0),
            0.75, 0.5, 100, Math.toRadians(40));
    Waypoint driveout2 = new Waypoint(transform(3.62, 152.4, noahOrigin), Math.toRadians(90), Math.toRadians(0),
            0.75, 0.5, 30, Math.toRadians(40));
    Waypoint strafe1 = new Waypoint(transform(27.48, 152.4, noahOrigin), Math.toRadians(90), Math.toRadians(90),
            0.5, 0.7, 20, Math.toRadians(40));
    Waypoint strafe2 = new Waypoint(transform(46.34, 152.4, noahOrigin), Math.toRadians(90), Math.toRadians(90),
            0.5, 0.7, 20, Math.toRadians(40));

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public void init() {
        super.init();
        drivetrain.setBrake(true);
        currentState = state.starting;
        lastState = state.starting;
        timeAtStateStart = System.currentTimeMillis();

    }

    public void start() {
        super.start();
        follower.initialize();
        localizer.setPosition(robotStartPosition, Math.toRadians(90));
    }


    boolean prepareLaunch = false;
    boolean goLaunch = false;
    boolean abortLaunch = false;
    boolean singleLaunch = false;

    double launchRPM;

    public void loop() {
        telemetry.addData("Auto state:", currentState);
        telemetry.addData("Path state:", follower.overallState);
        telemetry.addData("Launcher state", launcher.currentState);
        super.loop();

        if (currentState == state.starting) {
            launchRPM = powershotRPM;
            prepareLaunch = true;
            if (timeElapsedInState() > 500) {
                follower.initialize();
                currentState = state.driveout1;
            }
        }
        else if (currentState == state.driveout1) {
            follower.goToWaypoint(driveout1, false);
            if (follower.overallState == Follower.pathState.passed) {
                follower.initialize();
                currentState = state.driveout2;
            }
        }

        else if (currentState == state.driveout2) {
            follower.goToWaypoint(driveout2, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause1;
                prepareLaunch = false;
            }
        }

        else if (currentState == state.pause1) {
            follower.goToWaypoint(driveout2, true);
            if (timeElapsedInState() > 500) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot1;
            }
        }


        else if (currentState == state.powershot1) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                follower.initialize();
                currentState = state.strafe1;
            }
        }

        else if (currentState == state.strafe1) {
            follower.goToWaypoint(strafe1, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause2;
            }
        }

        else if (currentState == state.pause2) {
            follower.goToWaypoint(strafe1, true);
            if (timeElapsedInState() > 500) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot2;
            }
        }

        else if (currentState == state.powershot2) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                follower.initialize();
                currentState = state.strafe2;
            }
        }

        else if (currentState == state.strafe2) {
            follower.goToWaypoint(strafe2, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause3;
            }
        }

        else if (currentState == state.pause3) {
            follower.goToWaypoint(strafe2, true);
            if (timeElapsedInState() > 500) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot3;
            }
        }

        else if (currentState == state.powershot3) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                follower.initialize();
                currentState = state.ringthings;

                abortLaunch = true;
            }
        }

        else if (currentState == state.ringthings) {
            abortLaunch = false;
            launchRPM = normalRPM;
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }

        lastState = currentState;
        updateStateMachines();

    }

    private void updateStateMachines() {
        launcher.update(launchRPM, prepareLaunch, goLaunch, abortLaunch, singleLaunch);
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }

    //Takes points that are relative to some other origin and changes them to be relative to the field origin (bottom left corner)
    public Point transform(double x, double y, Point relativeTo) {
        return new Point (relativeTo.x + x, relativeTo.y + y);
    }

}

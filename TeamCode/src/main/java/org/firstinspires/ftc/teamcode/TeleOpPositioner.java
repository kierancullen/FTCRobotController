package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

//Handles auto-aiming during teleop

public class TeleOpPositioner {

    enum state {
        starting,
        navigating,
        floating,
        disabled
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public  TeleOpPositioner () {
        //nothing happens here
    }

    public void initialize() {
        currentState = state.disabled;
        lastState = state.disabled;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void update (Drivetrain drivetrain, Localizer localizer, Follower follower, Gamepad controller, boolean toggleNavigation, Point target) {
        if (currentState == state.starting) {
            follower.initialize(); //Starting a new move, so the states have to be reset
            drivetrain.setVelocityFromGamepad(controller);
            if (toggleNavigation) {
                currentState = state.navigating;
            }
            else {
                currentState = state.disabled;
            }
        }
        else if (currentState == state.navigating) {
            double angleToTarget = follower.angleTo(target);
            follower.goToWaypoint(new Waypoint(new Point (localizer.robotPosition.x, localizer.robotPosition.y), angleToTarget, 0, 0, 1.0, 40, Math.toRadians(40)), true);
            if (!gamepadAllZero(controller)) {
                currentState = state.floating;
            }
            else if (!toggleNavigation) {
                currentState = state.disabled;
            }
        }

        else if (currentState == state.floating) {
            drivetrain.setVelocityFromGamepad(controller);
            if (gamepadAllZero(controller)) {
                currentState = state.starting; //Auto-restart from this state
            }
            else if (!toggleNavigation) {
                currentState = state.disabled;
            }
        }

        else if (currentState == state.disabled) {
            drivetrain.setVelocityFromGamepad(controller);
            if (toggleNavigation) {
                currentState = state.starting;
            }
        }

        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }

    private boolean gamepadAllZero(Gamepad gamepad) {
        return Math.abs(gamepad.left_stick_y) == 0 && Math.abs(gamepad.right_stick_y) == 0
                && Math.abs(gamepad.right_trigger) == 0 && Math.abs(gamepad.left_trigger) == 0;
    }



}

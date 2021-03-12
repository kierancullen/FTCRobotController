package org.firstinspires.ftc.teamcode.Legacy;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Follower;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.Waypoint;

//A state machine that handles auto-aiming during teleop

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
        //nothing actually happens here
    }

    public void initialize() {
        currentState = state.disabled;
        lastState = state.disabled;
        timeAtStateStart = System.currentTimeMillis();
    }

    public void update (Drivetrain drivetrain, Localizer localizer, Follower follower, Gamepad controller, boolean toggleNavigation, Point target) {
        if (currentState == state.starting) {
            follower.initialize(); //Starting a new move, so the follower states have to be reset
            drivetrain.setVelocityFromGamepad(controller); //We're not actually navigating anywhere yet, so keep allowing gamepad control
            if (toggleNavigation) { //If we still want to start navigating, go to that state
                currentState = state.navigating;
            }
            else { //If we don't want to start navigating, go to the disabled state
                currentState = state.disabled;
            }
        }
        else if (currentState == state.navigating) {
            double angleToTarget = follower.angleTo(target); //Get the angle to where we want to aim (the goal
            //Tell our follower to navigate the robot to a point that's cour current position, but with the angle pointing at the target
            follower.goToWaypoint(new Waypoint(new Point (localizer.robotPosition.x, localizer.robotPosition.y), angleToTarget, 0, 0, 1.0, 40, Math.toRadians(40)), true);
            if (!gamepadAllZero(controller)) { //If we start using the gamepad, go to the floating state
                currentState = state.floating;
            }
            else if (!toggleNavigation) { //If we turn off navigation, go to the disabled state
                currentState = state.disabled;
            }
        }

        else if (currentState == state.floating) {
            drivetrain.setVelocityFromGamepad(controller); //Allow gamepad control now
            if (gamepadAllZero(controller)) {
                currentState = state.starting; //Auto-restart navigation from this state if we stop using the gamepad
            }
            else if (!toggleNavigation) {
                currentState = state.disabled;
            }
        }

        else if (currentState == state.disabled) {
            drivetrain.setVelocityFromGamepad(controller); //Allow gamepad control
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

    private boolean gamepadAllZero(Gamepad gamepad) { //Checks if any of our drive controls are being pressed
        return Math.abs(gamepad.left_stick_y) == 0 && Math.abs(gamepad.right_stick_y) == 0
                && Math.abs(gamepad.right_trigger) == 0 && Math.abs(gamepad.left_trigger) == 0;
    }



}

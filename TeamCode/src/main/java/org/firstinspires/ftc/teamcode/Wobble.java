package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//A class that represents the wobble grabber, with a state machine

public class Wobble {

    private Servo grabLeft;
    private Servo grabRight;
    private Servo tiltLeft;
    private Servo tiltRight;

    final double stowedPos = 0;
    final double matchStartPos = 0;
    final double floatPos = 0;
    final double downPos = 0;

    final double grabClosePos = 0;
    final double grabOpenPos = 0;

    private boolean grabbingNow;
    private boolean lastToggleGrab;

    enum state {
        matchStart,
        stowed,
        floating,
        down,
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public Wobble(Servo grabLeft, Servo grabRight, Servo tiltLeft, Servo tiltRight) {
        this.grabLeft = grabLeft;
        this.grabRight = grabRight;
        this.tiltLeft = tiltLeft;
        this.tiltRight = tiltRight;
    }

    //Call this once to get everything set up
    public void initialize() {
        currentState = state.stowed;
        lastState = state.stowed;
        timeAtStateStart = System.currentTimeMillis();
        grabbingNow = false;
    }

    //Call this in the opmode loop
    public void update(boolean prepare, boolean toggleGrab, boolean autoDrop, boolean wallDrop) {
        if (toggleGrab == true && lastToggleGrab == false && currentState == (state.down)) {
            grabbingNow = !grabbingNow;
        }

        lastToggleGrab = toggleGrab;

        if (currentState == state.stowed) {
            grabLeft.setPosition(grabOpenPos);
            grabRight.setPosition(grabOpenPos);
            tiltLeft.setPosition(stowedPos);
            tiltRight.setPosition(stowedPos);

            if (prepare && timeElapsedInState() > 500) {
                currentState = state.down;
            }
        }

        else if (currentState == state.down) {
            tiltLeft.setPosition(downPos);
            tiltRight.setPosition(downPos);
            if (grabbingNow) {
                grabLeft.setPosition(grabClosePos);
                grabRight.setPosition(grabClosePos);
            }
            else {
                grabLeft.setPosition(grabOpenPos);
                grabRight.setPosition(grabOpenPos);
            }

            if (prepare && timeElapsedInState() > 500) {
                if (grabbingNow) {
                    currentState = state.floating;
                }
                else {
                    currentState = state.stowed;
                }
            }

        }

        else if (currentState == state.floating) {
            tiltLeft.setPosition(floatPos);
            tiltRight.setPosition(floatPos);
            if (!grabbingNow && timeElapsedInState() > 500) {
                currentState = state.stowed;
            }
            else if (prepare && timeElapsedInState() > 500) {
                currentState = state.down;
                grabbingNow = false;
            }
        }

        else if (currentState == state.matchStart) {
            grabLeft.setPosition(grabClosePos);
            grabRight.setPosition(grabClosePos);
            tiltLeft.setPosition(matchStartPos);
            tiltRight.setPosition(matchStartPos);
            //Getting out of this state happens only in autonomous, so it can just be done by changing currentState
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

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

//A class that represents the wobble grabber, with a state machine

public class Wobble {

    private Servo grabLeft;
    private Servo grabRight;
    private Servo tiltLeft;
    private Servo tiltRight;

    final double stowedPos = 0.2;
    final double matchStartPos = 0.29;
    final double floatPos = 0.45;
    final double floatPosAuto = 0.7;
    final double downPos = 0.85;

    final double grabClosePos = 0.35;
    final double grabOpenPos = 1.0;

    public boolean grabbingNow;
    private boolean lastToggleGrab;

    enum state {
        matchStartOpen,
        matchStartClosed,
        stowed,
        floating,
        floatingAuto,
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
    public void update(boolean prepare, boolean toggleGrab) {
        if (toggleGrab == true && lastToggleGrab == false && (currentState == (state.down) || currentState == (state.floating))) {
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
            if (grabbingNow) {
                grabLeft.setPosition(grabClosePos);
                grabRight.setPosition(grabClosePos);
            }
            else {
                grabLeft.setPosition(grabOpenPos);
                grabRight.setPosition(grabOpenPos);
            }
            if (prepare && timeElapsedInState() > 500) {
                currentState = state.down;
                grabbingNow = false;
            }
        }
        else if (currentState == state.floatingAuto) {
            tiltLeft.setPosition(floatPosAuto);
            tiltRight.setPosition(floatPosAuto);
        }

        else if (currentState == state.matchStartOpen) {
            grabLeft.setPosition(grabOpenPos);
            grabRight.setPosition(grabOpenPos);
            tiltLeft.setPosition(matchStartPos);
            tiltRight.setPosition(matchStartPos);
            //Getting out of this state happens only in autonomous, so it can just be done by changing currentState manually
        }

        else if (currentState == state.matchStartClosed) {
            grabLeft.setPosition(grabClosePos);
            grabRight.setPosition(grabClosePos);
            tiltLeft.setPosition(matchStartPos);
            tiltRight.setPosition(matchStartPos);
            //Getting out of this state happens only in autonomous, so it can just be done by changing currentState manually
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

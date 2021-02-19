package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private static double tiltServoUp = 0.12;
    private static double tiltServoDown = 0;
    private static double pushServoIn = 0.27;
    private static double pushServoOut = 0.215;
    private static long strokeTime = 70; // In one direction (in/out time assumed same)

    private BangBangFlywheel flywheel;

    private Servo tiltServo;
    private Servo pushServo;

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    private int disksRemaining;

    enum state {
        load,
        launchReady,
        launchPush,
        launchRetract
    }

    public Launcher(DcMotor flywheelMotor, Servo tiltServo, Servo pushServo) {

        this.flywheel = new BangBangFlywheel(flywheelMotor, 28);
        this.tiltServo = tiltServo;
        this.pushServo = pushServo;
    }

    public void initialize() {
        currentState = Launcher.state.load;
        lastState = Launcher.state.load;
        timeAtStateStart = System.currentTimeMillis();

        disksRemaining = 0;
    }
    
    public void update(double launchRPM, boolean prepareLaunch, boolean goLaunch, boolean abort, boolean testLaunch) {
        flywheel.update();

        if (currentState == state.load) {
            tiltServo.setPosition(tiltServoDown);
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.stop();
            if (prepareLaunch) {
                currentState = state.launchReady;
                disksRemaining = 3;
            }
        }

        else if (currentState == state.launchReady) {
            tiltServo.setPosition(tiltServoUp);
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (goLaunch && flywheel.currentState == BangBangFlywheel.state.running) {
                currentState = state.launchPush;
            }
            else if (testLaunch && flywheel.currentState == BangBangFlywheel.state.running) {
                currentState = state.launchPush;
                disksRemaining = 1;
            }
            else if (abort) {
                currentState = state.load;
            }
        }
        else if (currentState == state.launchPush) {
            tiltServo.setPosition(tiltServoUp);
            pushServo.setPosition(pushServoIn);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > strokeTime) {
                currentState = state.launchRetract;
                disksRemaining -= 1;
            }
            else if (abort) {
                currentState = state.load;
            }
        }
        else if (currentState == state.launchRetract) {
            tiltServo.setPosition(tiltServoUp);
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > strokeTime && disksRemaining != 0) {
                currentState = state.launchPush;
            }
            else if (timeElapsedInState() > strokeTime && disksRemaining == 0) {
                currentState = state.load;
            }
            else if (abort) {
                currentState = state.load;
            }
        }

        // =====================================

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

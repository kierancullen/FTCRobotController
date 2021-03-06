package org.firstinspires.ftc.teamcode.Legacy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BangBangFlywheel;

public class TurretController {

    private static double TILTSERVO_UP = 0;
    private static double TILTSERVO_DOWN = 0;
    private static double PUSHSERVO_IN = 0;
    private static double PUSHSERVO_OUT = 0;
    private static long PUSHSERVO_STROKE_MS = 500; // In one direction (in/out time assumed same)
    private static double BLOCKSERVO_IN = 0;
    private static double BLOCKSERVO_OUT = 0;
    private static long BLOCKSERVO_STROKE_MS = 0;

    private WraparoundRotator rotator;
    private BangBangFlywheel flywheel;

    private Servo tiltServo;
    private Servo pushServo;
    private Servo blockServo;

    private TurretState state = TurretState.LOAD_PREP1;
    private TurretState lastState;
    private long timeAtStateStart;

    private int disksRemaining;

    enum TurretState {
        LOAD_PREP1,
        LOAD,
        LAUNCH_PREP2,
        LAUNCH_PREP1,
        LAUNCH_READY,
        LAUNCH_PUSH,
        LAUNCH_RETRACT
    }

    public TurretController(DcMotor rotatorMotor, DcMotor flywheelMotor,
                            Servo tiltServo, Servo pushServo, Servo blockServo) {

        this.rotator = new WraparoundRotator(rotatorMotor);
        this.flywheel = new BangBangFlywheel(flywheelMotor, 28);
        this.tiltServo = tiltServo;
        this.pushServo = pushServo;
        this.blockServo = blockServo;
    }

    public void tick(double launchAngle, double launchRPM,
                     boolean prepareLaunch, boolean prepareLoad, boolean goLaunch) {

        if (prepareLoad && (state != TurretState.LOAD_PREP1 && state != TurretState.LOAD)) {
            state = TurretState.LOAD_PREP1;
        }
        else if (prepareLaunch && (state == TurretState.LOAD_PREP1 || state == TurretState.LOAD)) {
            state = TurretState.LAUNCH_PREP2;
            disksRemaining = 3;
        }

        // =====================================

        if (state == TurretState.LOAD_PREP1) {
            tiltServo.setPosition(TILTSERVO_DOWN);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.stop();
            if (!rotator.isBusy()) {
                state = TurretState.LOAD;
            }
        }
        else if (state == TurretState.LOAD) {
            tiltServo.setPosition(TILTSERVO_DOWN);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_IN);
            flywheel.setTargetRPM(launchRPM);
            flywheel.stop();
        }
        else if (state == TurretState.LAUNCH_PREP2) {
            tiltServo.setPosition(TILTSERVO_DOWN);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > BLOCKSERVO_STROKE_MS) {
                state = TurretState.LAUNCH_PREP1;
            }
        }
        else if (state == TurretState.LAUNCH_PREP1) {
            tiltServo.setPosition(TILTSERVO_UP);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (!rotator.isBusy()) {
                state = TurretState.LAUNCH_READY;
            }
        }
        else if (state == TurretState.LAUNCH_READY) {
            tiltServo.setPosition(TILTSERVO_UP);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (goLaunch && flywheel.currentState == BangBangFlywheel.state.running) {
                state = TurretState.LAUNCH_PUSH;
                disksRemaining -= 1;
            }
        }
        else if (state == TurretState.LAUNCH_PUSH) {
            tiltServo.setPosition(TILTSERVO_UP);
            pushServo.setPosition(PUSHSERVO_IN);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > PUSHSERVO_STROKE_MS) {
                state = TurretState.LAUNCH_RETRACT;
            }
        }
        else if (state == TurretState.LAUNCH_RETRACT) {
            tiltServo.setPosition(TILTSERVO_UP);
            pushServo.setPosition(PUSHSERVO_OUT);
            blockServo.setPosition(BLOCKSERVO_OUT);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > PUSHSERVO_STROKE_MS && disksRemaining != 0) {
                state = TurretState.LAUNCH_READY;
            }
            else if (timeElapsedInState() > PUSHSERVO_STROKE_MS && disksRemaining == 0) {
                state = TurretState.LOAD_PREP1;
            }
        }

        // =====================================

        if (state != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = state;
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }
}

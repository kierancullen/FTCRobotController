package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private DcMotorEx intakeMotor;
    final double PPR = 28.0 * 3.7;
    final double stallThreshold = 100; //in RPM
    final long reverseTime = 1000;

    final double forwardPower = 0.95;
    final double reversePower = -0.3;

    enum state {
        running,
        reversing,
        unjamming,
        stopped
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public Intake (DcMotor intakeMotor) {
        this.intakeMotor = (DcMotorEx)intakeMotor;
    }

    public void initialize() {
        currentState = state.stopped;
        lastState = state.stopped;
        timeAtStateStart = System.currentTimeMillis();

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update(boolean trigger, boolean reverse) {
        double currentVelocity = (intakeMotor.getVelocity() / PPR * 60.0); //in RPM
        if (currentState == state.running) {
            intakeMotor.setPower(forwardPower);
            if (reverse) {
                currentState = state.reversing;
            }
            else if (trigger && (timeElapsedInState() > 500)) {
                currentState = state.stopped;
            }
            else if (currentVelocity < stallThreshold && (timeElapsedInState() > 500)) { //some quick stall detection
                currentState = state.unjamming;
            }

        }

        else if (currentState == state.reversing) {
            intakeMotor.setPower(reversePower);
            if (!reverse) {
                currentState = state.running;
            }
        }

        else if (currentState == state.stopped) {
            intakeMotor.setPower(0);
            if (trigger && (timeElapsedInState() > 500)) {
                currentState = state.running;
            }
        }

        else if (currentState == state.unjamming) {
            intakeMotor.setPower(reversePower);
            if (timeElapsedInState() > reverseTime) {
                currentState = state.running;
            }
            else if (trigger) {
                currentState = state.stopped;
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


}

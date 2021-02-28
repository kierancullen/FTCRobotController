package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//A class that represents the intake, with a state machine

public class Intake {

    private DcMotorEx intakeMotor;
    private DistanceSensor distance;
    final double PPR = 28.0 * 3.7;
    final double stallThreshold = 100; //in RPM
    final long reverseTime = 1000;

    final double forwardPower = 0.5;
    final double reversePower = -0.95;

    public int ringCount;
    final double countThreshold = 7.0; //Distance the sensor must fall below in order to count a ring
    final long pause = 100; //Time to make sure we don't detect the same ring twice
    private long timeAtLastRing;

    enum state {
        running,
        reversing,
        unjamming,
        stopped
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public Intake (DcMotor intakeMotor, DistanceSensor distance) {
        this.intakeMotor = (DcMotorEx)intakeMotor;
        this.distance = distance;
    }

    //Call this once to get everything set up
    public void initialize() {
        currentState = state.stopped;
        lastState = state.stopped;
        timeAtStateStart = System.currentTimeMillis();

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ringCount = 0;
    }

    //Call this in the opmode loop
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
            //Count up if the distance sensor sees a ring and it hasn't been too little time since it last triggered
            if (distance.getDistance(DistanceUnit.CM) < countThreshold && (System.currentTimeMillis() - timeAtLastRing > pause)) {
                ringCount++;
                timeAtLastRing = System.currentTimeMillis();
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

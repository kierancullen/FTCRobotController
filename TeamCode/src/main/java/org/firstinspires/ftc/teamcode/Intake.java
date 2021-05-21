package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//A class that represents the intake, with a state machine

public class Intake {

    private DcMotorEx intakeMotor;
    private CRServo feedLeft;
    private CRServo feedRight;

    final double PPR = 28.0 * 3.7;
    final double stallThreshold = 100; //in RPM
    final long reverseTime = 1000;

    final double forwardPower = 0.3;
    final double reversePower = -0.95;

    final double feedForwardPower = 0.3; // TODO fill in
    final double feedReversePower = -0.95; // TODO fill in

    final double gateStowPos = 0.175;
    final double gateRaisedPos = 0.7;
    final double gateAutoPos = 0.30;

    final double deflectorDownPos = 0.6;
    final double deflectorVertPos = 0.3;
    final double deflectorStowPos = 0.02;

    public int ringCount;
    public int ringCountReal;
    final double countThresholdLower = 6.0;
    final double countThresholdUpper = 8.0;
    boolean crossedDown;

    public boolean gate;
    public boolean gatePushing;
    public boolean deflectors;
    public boolean deflectorsStowed;


    enum state {
        running,
        reversing,
        unjamming,
        stopped
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public Intake (DcMotor intakeMotor, CRServo feedLeft, CRServo feedRight) {
        this.intakeMotor = (DcMotorEx)intakeMotor;
        this.feedLeft = feedLeft;
        this.feedRight = feedRight;
    }

    //Call this once to get everything set up
    public void initialize() {
        currentState = state.stopped;
        lastState = state.stopped;
        timeAtStateStart = System.currentTimeMillis();

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Call this in the opmode loop
    public void update(boolean trigger, boolean reverse) {

        double currentVelocity = (intakeMotor.getVelocity() / PPR * 60.0); //in RPM
        if (currentState == state.running) {
            intakeMotor.setPower(forwardPower);
            feedLeft.setPower(feedForwardPower);
            feedRight.setPower(feedForwardPower);
            if (reverse) {
                currentState = state.reversing;
            }
            else if (trigger && (timeElapsedInState() > 500)) {
                currentState = state.stopped;
            }
        }

        else if (currentState == state.reversing) {
            intakeMotor.setPower(reversePower);
            feedLeft.setPower(feedReversePower);
            feedRight.setPower(feedReversePower);
            if (!reverse) {
                currentState = state.running;
            }
        }

        else if (currentState == state.stopped) {
            intakeMotor.setPower(0);
            feedLeft.setPower(0);
            feedRight.setPower(0);
            if (trigger && (timeElapsedInState() > 500)) {
                currentState = state.running;
            }
            if (reverse) {
                intakeMotor.setPower(reversePower);
                feedLeft.setPower(feedReversePower);
                feedRight.setPower(feedReversePower);
            }
        }

        else if (currentState == state.unjamming) {
            intakeMotor.setPower(reversePower);
            feedLeft.setPower(feedReversePower);
            feedRight.setPower(feedReversePower);
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

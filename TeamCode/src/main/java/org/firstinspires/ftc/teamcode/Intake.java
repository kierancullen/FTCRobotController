package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//A class that represents the intake, with a state machine

public class Intake {

    private DcMotorEx intakeMotor;
    public DistanceSensor distance;
    private Servo gateLeft;
    private Servo gateRight;
    private Servo deflectorLeft;
    private Servo deflectorRight;

    final double PPR = 28.0 * 3.7;
    final double stallThreshold = 100; //in RPM
    final long reverseTime = 1000;

    final double forwardPower = 0.3;
    final double reversePower = -0.95;

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

    public Intake (DcMotor intakeMotor, DistanceSensor distance, Servo gateLeft, Servo gateRight, Servo deflectorLeft, Servo deflectorRight) {
        this.intakeMotor = (DcMotorEx)intakeMotor;
        this.distance = distance;
        this.gateLeft = gateLeft;
        this.gateRight = gateRight;
        this.deflectorLeft = deflectorLeft;
        this.deflectorRight = deflectorRight;
    }

    //Call this once to get everything set up
    public void initialize() {
        currentState = state.stopped;
        lastState = state.stopped;
        timeAtStateStart = System.currentTimeMillis();

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ringCount = 0;

        gateLeft.setPosition(gateStowPos);
        gateRight.setPosition(gateStowPos);
        gate = true;

        deflectorLeft.setPosition(deflectorStowPos);
        deflectorRight.setPosition(deflectorStowPos);
        deflectors = false;
        deflectorsStowed = false;

    }

    //Call this in the opmode loop
    public void update(boolean trigger, boolean reverse) {
        ringCountReal = ringCount/2;
        if (gate) {
            if (gatePushing) {
                gateLeft.setPosition(gateAutoPos);
                gateRight.setPosition(gateAutoPos);
            }
            else {
                gateLeft.setPosition(gateRaisedPos);
                gateRight.setPosition(gateRaisedPos);
            }

        }
        else {
            gateLeft.setPosition(gateStowPos);
            gateRight.setPosition(gateStowPos);
        }
        if (deflectorsStowed) {
            deflectorLeft.setPosition(deflectorStowPos);
            deflectorRight.setPosition(deflectorStowPos);
        }
        else {
            if (deflectors) {
                deflectorLeft.setPosition(deflectorDownPos);
                deflectorRight.setPosition(deflectorDownPos);
            }
            else {
                deflectorLeft.setPosition(deflectorVertPos);
                deflectorRight.setPosition(deflectorVertPos);
            }
        }



        if (distance.getDistance(DistanceUnit.CM) < countThresholdLower) {
            crossedDown = true;
        }
        if (distance.getDistance(DistanceUnit.CM) > countThresholdUpper) {
            if (crossedDown) {
                ringCount++;
                crossedDown = false;
            }
        }


        double currentVelocity = (intakeMotor.getVelocity() / PPR * 60.0); //in RPM
        if (currentState == state.running) {
            intakeMotor.setPower(forwardPower);
            if (reverse) {
                currentState = state.reversing;
            }
            else if (trigger && (timeElapsedInState() > 500)) {
                currentState = state.stopped;
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

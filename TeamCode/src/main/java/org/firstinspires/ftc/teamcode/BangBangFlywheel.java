package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BangBangFlywheel {

    public enum state {
        stopped,
        starting,
        running
    }

    private DcMotorEx flywheelMotor;
    public state currentState;
    private double targetRPM;
    private double PPR;

    public BangBangFlywheel(DcMotor flywheelMotor, double PPR) {
        this.flywheelMotor = (DcMotorEx) flywheelMotor;
        this.PPR = PPR;
        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = Math.abs(targetRPM);
    }

    public void start() {
        if (this.currentState == state.stopped) {
            this.currentState = state.starting;
        }
    }

    public void stop() {
        if (this.currentState != state.stopped) {
            this.currentState = state.stopped;
        }
    }

    public void update() {
        if (this.currentState == state.stopped) {
            this.flywheelMotor.setPower(0);
        }
        else if (this.currentState == state.starting) {
            this.flywheelMotor.setPower(1);
            if (this.getCurrentRPM() >= this.targetRPM) {
                this.currentState = state.running;
            }
        }
        else if (this.currentState == state.running) {
            if (this.getCurrentRPM() >= this.targetRPM) {
                this.flywheelMotor.setPower(0);
            }
            else {
                this.flywheelMotor.setPower(1);
            }
        }
    }

    public double getCurrentRPM() {
        return Math.abs(this.flywheelMotor.getVelocity()) / this.PPR * 60.0;
    }

}

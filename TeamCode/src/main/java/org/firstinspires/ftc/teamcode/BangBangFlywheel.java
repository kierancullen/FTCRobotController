package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BangBangFlywheel {

    public enum FlywheelState {STOPPED, STARTING, RUNNING};

    private DcMotorEx motor;
    private FlywheelState state;
    private double targetRPM;
    private double pulsesPerRevolution;

    public BangBangFlywheel(DcMotor motor, double pulsesPerRevolution) {
        this.motor = (DcMotorEx)motor;
        this.pulsesPerRevolution = pulsesPerRevolution;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = Math.abs(targetRPM);
    }

    public void start() {
        this.state = FlywheelState.STARTING;
        this.tick();
    }

    public void stop() {
        this.state = FlywheelState.STOPPED;
        this.tick();
    }

    public void tick() {
        if (this.state == FlywheelState.STOPPED) {
            this.motor.setPower(0);
        }
        else if (this.state == FlywheelState.STARTING) {
            this.motor.setPower(1);
            if (this.getCurrentRPM() >= this.targetRPM) {
                this.state = FlywheelState.RUNNING;
            }
        }
        else if (this.state == FlywheelState.RUNNING) {
            if (this.getCurrentRPM() >= this.targetRPM) {
                this.motor.setPower(0);
            }
            else {
                this.motor.setPower(1);
            }
        }
    }

    public FlywheelState getState() {
        return this.state;
    }

    private double getCurrentRPM() {
        return Math.abs(this.motor.getVelocity()) / this.pulsesPerRevolution * 60.0;
    }

}

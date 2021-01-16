package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class WraparoundRotator {

    private DcMotor motor;

    private double hwSetpoint;
    private double targetAngle;

    public WraparoundRotator(DcMotor motor) {
        this.motor = motor;
    }

    public void setTargetAngle(double newTargetAngle) {

    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isBusy() {
        return false;//motor.isBusy();
    }
}

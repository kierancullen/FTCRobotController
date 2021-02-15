package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Drivetrain {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public boolean gamepadControl;

    final private double differentialMax = 1.0;
    final private double strafeMax = 1.0;

    private Point translateVelocity;
    private double turnVelocity;

    Gamepad controller;

    public Drivetrain(DcMotor topLeft, DcMotor topRight, DcMotor backLeft, DcMotor backRight, Gamepad controller) {
        this.frontLeft = topLeft;
        this.frontRight = topRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.controller = controller;

        gamepadControl = true;

        translateVelocity = new Point();
        translateVelocity.x = 0;
        translateVelocity.y = 0;
        turnVelocity = 0;
    }

    public void update() {

        if (gamepadControl) {
            setVelocityFromGamepad();
        }

        setPowers();
    }

    private void setPowers() {
        double tl_power_raw = translateVelocity.y-turnVelocity+translateVelocity.x;
        double bl_power_raw = translateVelocity.y-turnVelocity- translateVelocity.x;
        double br_power_raw = -(translateVelocity.y+turnVelocity+translateVelocity.x);
        double tr_power_raw = -(translateVelocity.y+turnVelocity-translateVelocity.x);


        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        frontLeft.setPower(tl_power_raw);
        backLeft.setPower(bl_power_raw);
        backRight.setPower(br_power_raw);
        frontRight.setPower(tr_power_raw);
    }

    private void setVelocityFromGamepad() {
        double horiz;
        double l;
        double r;

        horiz = 1.5 * (controller.right_trigger * differentialMax) - (controller.left_trigger * differentialMax);
        l = -controller.left_stick_y * strafeMax;
        r = -controller.right_stick_y * strafeMax;

        translateVelocity.x = horiz;
        translateVelocity.y = (l + r) / 2.0;
        turnVelocity = (r - l) / 3;
    }

    public void setBrake(boolean brake) {
        if (brake) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


}
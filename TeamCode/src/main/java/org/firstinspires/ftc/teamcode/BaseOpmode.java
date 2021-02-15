package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BaseOpmode extends OpMode {

    Drivetrain drivetrain;
    Localizer localizer;

    final double odometryTicksPerUnit = (360 * 4) / (5.8 * Math.PI);
    final double odometryLeftRadius = (6 * 2.54) + (4.39); //4.473
    final double odometryRightRadius = (6 * 2.54) + (4.39);
    final double odometryCenterRadius = (6.75 * 2.54) - (4.39);

    public void init() {

        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        drivetrain = new Drivetrain(tl, tr, bl, br, gamepad1);
        Odometer left = new Odometer(br, true, odometryTicksPerUnit, odometryLeftRadius);
        Odometer center = new Odometer(bl, true, odometryTicksPerUnit, odometryCenterRadius);
        Odometer right = new Odometer(tr, false, odometryTicksPerUnit, odometryRightRadius);

        localizer = new Localizer(left, right, center);
    }


    public void start() {
        localizer.setPosition(new Point(0,0), 0); //Make sure we're at the origin
    }


    public void loop() {
        drivetrain.update();
        localizer.update();

        telemetry.update();
    }
}

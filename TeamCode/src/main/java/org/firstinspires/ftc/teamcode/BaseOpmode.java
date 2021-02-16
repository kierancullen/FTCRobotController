package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BaseOpmode extends OpMode {

    Drivetrain drivetrain;
    Localizer localizer;

    final double odometryTicksPerUnit = (360 * 4) / (5.8 * Math.PI);
    final double odometryLeftRadius = 19.674; //(6 * 2.54) + (4.4); //4.473
    final double odometryRightRadius = 19.674; //(6 * 2.54) + (4.4);
    final double odometryCenterRadius = (6.75 * 2.54) - (4.4);

    final double odometryRightBias = 0.9975;
    final double odometryLeftBias = 1.0;
    final double odometryCenterBias = 1.0;

    public void init() {

        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        drivetrain = new Drivetrain(tl, tr, bl, br, gamepad1);
        Odometer left = new Odometer(br, true, odometryTicksPerUnit, odometryLeftRadius, odometryLeftBias);
        Odometer center = new Odometer(bl, true, odometryTicksPerUnit, odometryCenterRadius, odometryCenterBias);
        Odometer right = new Odometer(tr, false, odometryTicksPerUnit, odometryRightRadius, odometryRightBias);

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

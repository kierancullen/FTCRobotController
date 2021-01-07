package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.text.DecimalFormat;

@TeleOp(name="LaunchTestBangBang")
public class LaunchTestBangBang extends OpMode {

    BangBangFlywheel flywheel;

    DcMotorEx launchMotor;
    double targetVelocity;

    final double PPR = 28.0; //Bare NeveRest pulses per revolution

    boolean lastY;
    boolean lastA;

    DecimalFormat df = new DecimalFormat("#.##");

    public void init() {
        launchMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "launch");
        flywheel = new BangBangFlywheel(launchMotor, PPR);
        flywheel.start();
    }

    public void loop() {

        telemetry.addData("Target velocity:", df.format(targetVelocity));
        telemetry.addData("Current hardware velocity:", df.format(launchMotor.getVelocity() / PPR * 60.0));
        telemetry.addData("Current encoder:", launchMotor.getCurrentPosition());

        flywheel.setTargetRPM(targetVelocity);
        flywheel.tick();

        if (lastY && !gamepad1.y) targetVelocity += 50;
        if (lastA && !gamepad1.a) targetVelocity -= 50;

        lastY = gamepad1.y;
        lastA = gamepad1.a;

    }

}

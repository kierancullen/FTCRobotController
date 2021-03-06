package org.firstinspires.ftc.teamcode.Legacy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.text.DecimalFormat;

@TeleOp(name="LaunchTest")
public class LaunchTest extends OpMode {

    DcMotorEx LaunchMotor;
    double targetVelocity;
    long lastEncoder;
    long lastTime;

    final double PPR = 28.0; //Bare NeveRest pulses per revolution

    boolean lastY;
    boolean lastA;


    DecimalFormat df = new DecimalFormat("#.##");

    public void init() {
        LaunchMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "launch");
        DecimalFormat df = new DecimalFormat("#.##");
        LaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        targetVelocity = 0;
        LaunchMotor.setVelocity(targetVelocity * PPR / 60.0);

        lastEncoder = LaunchMotor.getCurrentPosition();
        lastTime = System.currentTimeMillis();

        MotorConfigurationType h;

        lastY = false;
        lastA = false;

    }

    double currentVelocity = 0;

    public void loop() {
        if (System.currentTimeMillis() - lastTime > 250) {
            currentVelocity = ((LaunchMotor.getCurrentPosition() - lastEncoder) / ((double)(System.currentTimeMillis() - lastTime))) * 1000.0 * 60.0 / PPR;
            lastEncoder = LaunchMotor.getCurrentPosition();
            lastTime = System.currentTimeMillis();
        }

        telemetry.addData("Target velocity:", df.format(targetVelocity));
        telemetry.addData("Current velocity:", df.format(currentVelocity));
        telemetry.addData("Current hardware velocity:", df.format(LaunchMotor.getVelocity() / PPR * 60.0));
        telemetry.addData("Current encoder:", LaunchMotor.getCurrentPosition());

        LaunchMotor.setVelocity(targetVelocity * PPR / 60.0);

        if (lastY && gamepad1.y == false) targetVelocity += 50;
        if (lastA && gamepad1.a == false) targetVelocity -= 50;

        lastY = gamepad1.y;
        lastA = gamepad1.a;

    }

}

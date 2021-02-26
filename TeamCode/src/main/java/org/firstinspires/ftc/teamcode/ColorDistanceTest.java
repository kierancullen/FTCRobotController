package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ColorDistanceTest")
public class ColorDistanceTest extends OpMode {

    private DistanceSensor intakeSensor;

    public void init() {
        intakeSensor = hardwareMap.get(DistanceSensor.class, "color");
    }

    public void loop() {
        telemetry.addData("Proximity value (cm):", intakeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}

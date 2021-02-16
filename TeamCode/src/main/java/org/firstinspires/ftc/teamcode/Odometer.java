package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Odometer {

    private DcMotor encoder;
    public boolean invert;

    public long lastResetValue;
    private long lastValue;
    public long currentValue;

    public double delta;
    public double totalDelta;

    public double ticksPerUnit;
    public double radius;
    public double bias;

    public Odometer (DcMotor encoder, boolean invert, double ticksPerUnit, double radius, double bias) {
        this.encoder = encoder;
        this.invert = invert;
        this.ticksPerUnit = ticksPerUnit;
        this.radius = radius;
        this.bias = bias;

        if (invert) lastResetValue = -encoder.getCurrentPosition();
        else lastResetValue = encoder.getCurrentPosition();
    }

    public void update() {
        if (invert) currentValue = (-encoder.getCurrentPosition());
        else currentValue = encoder.getCurrentPosition();

        delta = bias *((double)(currentValue-lastValue) / ticksPerUnit); //How much we moved, in real units
        totalDelta = bias *((double)(currentValue-lastResetValue) / ticksPerUnit); //How much we moved since the beginning, in real units

        lastValue = currentValue;

    }


}

package org.firstinspires.ftc.teamcode.Legacy;
import com.qualcomm.robotcore.hardware.DcMotor;

//A class that represents a single odometry wheel

public class OdometerOld {

    private DcMotor encoder;
    public boolean invert; //reverses direction

    public long lastResetValue;
    private long lastValue;
    public long currentValue;

    public double delta;
    public double totalDelta;

    public double deltaRaw;
    public double totalDeltaRaw;

    public double ticksPerUnit;
    public double radius;
    public double bias;

    public OdometerOld (DcMotor encoder, boolean invert, double ticksPerUnit, double radius, double bias) {
        this.encoder = encoder;
        this.invert = invert;
        this.ticksPerUnit = ticksPerUnit;
        this.radius = radius;
        this.bias = bias;

        if (invert) lastResetValue = -encoder.getCurrentPosition();
        else lastResetValue = encoder.getCurrentPosition();
    }

    public void update() {
        //Fetch our current value from the encoder
        if (invert) currentValue = (-encoder.getCurrentPosition());
        else currentValue = encoder.getCurrentPosition();

        delta = bias *((double)(currentValue-lastValue) / ticksPerUnit); //How much we moved, in real units
        totalDelta = bias *((double)(currentValue-lastResetValue) / ticksPerUnit); //How much we moved since the beginning, in real units
        deltaRaw = bias * (currentValue-lastValue); //How much we moved, in encoder ticks
        totalDeltaRaw = bias *(currentValue-lastResetValue); //How much we moved since the beginning, in encoder ticks

        lastValue = currentValue;

    }


}

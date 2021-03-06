package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

//A class that represents a single odometry wheel

public class Odometer {

    private DcMotor encoder;
    public boolean invert; //reverses direction

    public long lastResetValue;
    private long lastValue;
    public long currentValue;

    public double deltaRaw;
    public double totalDeltaRaw;

    public double bias;

    public Odometer (DcMotor encoder, boolean invert, double bias) {
        this.encoder = encoder;
        this.invert = invert;
        this.bias = bias;

        if (invert) lastResetValue = -encoder.getCurrentPosition();
        else lastResetValue = encoder.getCurrentPosition();
    }

    public void update() {
        //Fetch our current value from the encoder
        if (invert) currentValue = (-encoder.getCurrentPosition());
        else currentValue = encoder.getCurrentPosition();

        deltaRaw = bias * (currentValue-lastValue); //How much we moved, in encoder ticks
        totalDeltaRaw = bias *(currentValue-lastResetValue); //How much we moved since the beginning, in encoder ticks

        lastValue = currentValue;

    }


}

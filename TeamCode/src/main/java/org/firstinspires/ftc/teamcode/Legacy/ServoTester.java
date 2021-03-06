package org.firstinspires.ftc.teamcode.Legacy;

import com.qualcomm.robotcore.hardware.Servo;

//A quick class that I was using to test an array of servos

public class ServoTester {
    Servo[] servos;
    double[] servoPowers;
    int currentServo;
    double currentPower;


    public ServoTester (Servo[] servos) {
        this.servos = servos;
        servoPowers = new double[servos.length];
        currentServo = 0;

        for (double power:servoPowers) {
            power = 0;
        }
    }

    public void update(boolean nextServo, boolean previousServo, boolean up, boolean down) {
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(servoPowers[i]);
        }
        currentPower = servoPowers[currentServo];
        if (up) servoPowers[currentServo]+= 0.005;
        else if (down) servoPowers[currentServo]-= 0.005;
        if (nextServo) currentServo++;
        else if (previousServo) currentServo--;

        if (currentServo < 0) currentServo = servos.length - 1;
        else if (currentServo >= servos.length) currentServo = 0;
    }
}

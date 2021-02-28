package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {

    Servo gateLeft;
    Servo gateRight;
    Servo deflectorLeft;
    Servo deflectorRight;
    ServoTester tester;

    public void init() {
        this.gateLeft = hardwareMap.get(Servo.class, "leftGate");
        this.gateRight = hardwareMap.get(Servo.class, "rightGate");
        this.deflectorLeft = hardwareMap.get(Servo.class, "leftDefl");
        this.deflectorRight = hardwareMap.get(Servo.class, "rightDefl");
        setServoExtendedRange(gateLeft, 500, 2500);
        setServoExtendedRange(gateRight, 500, 2500);
        setServoExtendedRange(deflectorLeft, 500, 2500);
        setServoExtendedRange(deflectorRight, 500, 2500);
        gateLeft.setDirection(Servo.Direction.REVERSE);
        deflectorRight.setDirection(Servo.Direction.REVERSE);

        tester = new ServoTester(new Servo[] {gateLeft, gateRight, deflectorLeft, deflectorRight});
    }

    public void loop() {
       tester.update(gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down);
       telemetry.addData("Servo:", tester.currentServo);
       telemetry.addData("Position:", tester.currentPower);

    }

    public void setServoExtendedRange (Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }
}

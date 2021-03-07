package org.firstinspires.ftc.teamcode.Legacy;

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
        this.gateLeft = hardwareMap.get(Servo.class, "grabLeft");
        this.gateRight = hardwareMap.get(Servo.class, "grabRight");
        this.deflectorLeft = hardwareMap.get(Servo.class, "wobbleTiltLeft");
        this.deflectorRight = hardwareMap.get(Servo.class, "wobbleTiltRight");
        setServoExtendedRange(gateLeft, 500, 2500);
        setServoExtendedRange(gateRight, 500, 2500);
        /*setServoExtendedRange(deflectorLeft, 500, 2500);
        setServoExtendedRange(deflectorRight, 500, 2500);*/

        tester = new ServoTester(new Servo[] {gateLeft, gateRight, deflectorLeft, deflectorRight});
    }

    boolean rightLast = false;
    boolean advance = false;
    public void loop() {
        if (gamepad1.dpad_right && !rightLast) {
            advance = true;
        }
        else {
            advance = false;
        }
        rightLast = gamepad1.dpad_right;
       tester.update(advance, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down);
       telemetry.addData("Servo:", tester.currentServo);
       telemetry.addData("Position:", tester.currentPower);

    }

    public void stop() {
        gateLeft.getController().pwmDisable();
        deflectorLeft.getController().pwmDisable();
    }

    public void setServoExtendedRange (Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }

}

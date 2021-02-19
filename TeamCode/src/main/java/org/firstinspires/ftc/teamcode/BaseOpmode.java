package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class BaseOpmode extends OpMode {

    Drivetrain drivetrain;
    Localizer localizer;
    Intake intake;
    Launcher launcher;
    SpeedTracker tracker;
    Follower follower;

    final double odometryTicksPerUnit = (360 * 4) / (5.8 * Math.PI);
    final double odometryLeftRadius = 19.674; //(6 * 2.54) + (4.4); //4.473
    final double odometryRightRadius = 19.674; //(6 * 2.54) + (4.4);
    final double odometryCenterRadius = (6.75 * 2.54) - (4.445);

    final double odometryRightBias = 0.9970;
    final double odometryLeftBias = 1.0;
    final double odometryCenterBias = 1.0;

    final double launchRPM = 3600;

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
        tracker = new SpeedTracker(localizer);
        follower = new Follower(localizer, drivetrain, tracker);

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intake = new Intake(intakeMotor);

        DcMotor launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        Servo tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        Servo pushServo = hardwareMap.get(Servo.class, "pushServo");
        tiltServo.setDirection(Servo.Direction.REVERSE);
        pushServo.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(tiltServo, 500, 2500);
        setServoExtendedRange(pushServo, 500, 2500);
        launcher = new Launcher(launcherMotor, tiltServo, pushServo);


    }


    public void start() {
        localizer.setPosition(new Point(0,0), 90); //Make sure we're at the origin
        intake.initialize();
        launcher.initialize();
        follower.initialize();
    }


    public void loop() {
        drivetrain.update();
        localizer.update();
        intake.update(gamepad1.right_bumper, gamepad1.left_bumper);
        launcher.update(launchRPM, gamepad1.dpad_up, gamepad1.y, gamepad1.a, gamepad1.b);
        telemetry.update();

        follower.goToSimple(new Point(0, 10), 0, 0.5, 0.5, Math.toDegrees(60), 0, true);
    }

    public void setServoExtendedRange (Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }
}

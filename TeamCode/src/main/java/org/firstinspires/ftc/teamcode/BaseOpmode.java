package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

//An opmode intended to be shared by both TeleOp and Autonomous
//Initializes all the objects for Intake, Launcher, odometry stuff, etc.

public class BaseOpmode extends OpMode {

    Drivetrain drivetrain;
    Localizer localizer;
    Intake intake;
    Launcher launcher;
    SpeedTracker tracker;
    Follower follower;

    final double odometryTicksPerUnit = (360 * 4) / (5.8 * Math.PI); //I'm considering a unit to be 1 cm
    final double odometryLeftRadius = 19.924;
    final double odometryRightRadius = 19.924;
    final double odometryCenterRadius = 12.45;

    //Use these to correct if an odometry wheel seems to just rotate more or less than another one
    final double odometryRightBias = 1.0;
    final double odometryLeftBias = 1.0;
    final double odometryCenterBias = 1.0;

    public void init() {

        //Create the drivetrain
        DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
        DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");

        drivetrain = new Drivetrain(tl, tr, bl, br);

        //Create the localizer and related classes
        Odometer left = new Odometer(tr, true, odometryTicksPerUnit, odometryLeftRadius, odometryLeftBias);
        Odometer center = new Odometer(bl, true, odometryTicksPerUnit, odometryCenterRadius, odometryCenterBias);
        Odometer right = new Odometer(br, false, odometryTicksPerUnit, odometryRightRadius, odometryRightBias);

        localizer = new Localizer(left, right, center);
        tracker = new SpeedTracker(localizer);
        follower = new Follower(localizer, drivetrain, tracker);

        //Create the intake
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "color");
        Servo gateLeft = hardwareMap.get(Servo.class, "leftGate");
        Servo gateRight = hardwareMap.get(Servo.class, "rightGate");
        setServoExtendedRange(gateLeft, 500, 2500);
        setServoExtendedRange(gateRight, 500, 2500);
        gateLeft.setDirection(Servo.Direction.REVERSE);
        intake = new Intake(intakeMotor, distance, gateLeft, gateRight);

        //Create the launcher
        DcMotor launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        Servo tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        Servo pushServo = hardwareMap.get(Servo.class, "pushServo");
        tiltServo.setDirection(Servo.Direction.REVERSE);
        pushServo.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(tiltServo, 500, 2500);
        setServoExtendedRange(pushServo, 500, 2500);
        launcher = new Launcher(launcherMotor, tiltServo, pushServo);

        intake.initialize();
        launcher.initialize();
        follower.initialize();
        drivetrain.setBrake(false);
    }


    public void start() {
        localizer.setPosition(new Point(0,0),  Math.toRadians(90)); //Make sure we're at the origin (TeleOp and Autonomous opmodes can change this if needed)
    }

    public void loop() {
        localizer.update();
        //Some debugging stuff
        telemetry.addData("localizer x:", localizer.robotPosition.x);
        telemetry.addData("localizer y:", localizer.robotPosition.y);
        telemetry.addData("Localizer angle", Math.toDegrees(localizer.robotAngle));
        telemetry.addData("Right wheel raw", localizer.right.totalDeltaRaw);
        telemetry.addData("Left wheel raw", localizer.left.totalDeltaRaw);
        telemetry.addData("Center wheel raw", localizer.center.totalDeltaRaw);
        drivetrain.update();
        telemetry.update();
    }

    //Used to set the GoBilda servos to the correct PWM range
    public void setServoExtendedRange (Servo servo, int min, int max) {
        ServoControllerEx controller = (ServoControllerEx) servo.getController();
        int servoPort = servo.getPortNumber();
        PwmControl.PwmRange range = new PwmControl.PwmRange(min, max);
        controller.setServoPwmRange(servoPort, range);
    }
}

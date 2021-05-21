package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Wobble wobble;
    PreconfigStorage storage;

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
        Odometer left = new Odometer(tr, true, odometryLeftBias);
        Odometer center = new Odometer(bl, true,  odometryCenterBias);
        Odometer right = new Odometer(br, false,  odometryRightBias);

        localizer = new Localizer(left, right, center);
        tracker = new SpeedTracker(localizer);
        follower = new Follower(localizer, drivetrain, tracker);

        //Create the intake
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        CRServo feedLeft = hardwareMap.get(CRServo.class, "feedLeft");
        CRServo feedRight = hardwareMap.get(CRServo.class, "feedRight");
        feedRight.setDirection(CRServo.Direction.REVERSE);

        intake = new Intake(intakeMotor, feedLeft, feedRight);

        //Create the launcher
        DcMotor launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        DcMotor turretMotor = hardwareMap.get(DcMotor.class, "turret");
        Servo pushServo = hardwareMap.get(Servo.class, "pushServo");
        pushServo.setDirection(Servo.Direction.REVERSE);
        setServoExtendedRange(pushServo, 500, 2500);
        launcher = new Launcher(launcherMotor, pushServo, turretMotor);

        //Create the wobble goal grabber
        Servo grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        Servo grabRight = hardwareMap.get(Servo.class, "grabRight");
        Servo wobbleTiltLeft = hardwareMap.get(Servo.class, "wobbleTiltLeft");
        Servo wobbleTiltRight = hardwareMap.get(Servo.class, "wobbleTiltRight");
        wobble = new Wobble (grabLeft, grabRight, wobbleTiltLeft, wobbleTiltRight);

        //Create the storage
        storage = new PreconfigStorage(hardwareMap.appContext);

        intake.initialize();
        launcher.initialize();
        wobble.initialize();
        follower.initialize();
        drivetrain.setBrake(false);
    }

    public void init_loop() {
        // Runs like loop() but in init; use to home the turret
        launcher.home();
    }

    public void start() {
        localizer.setPosition(new Point(0,0),  Math.toRadians(90)); //Make sure we're at the origin (TeleOp and Autonomous opmodes can change this if needed)
    }

    public void loop() {
        localizer.update();
        tracker.update();
        //Some debugging stuff
        telemetry.addData("localizer x:", localizer.robotPosition.x);
        telemetry.addData("localizer y:", localizer.robotPosition.y);
        telemetry.addData("localizer angle", Math.toDegrees(localizer.robotAngle));
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

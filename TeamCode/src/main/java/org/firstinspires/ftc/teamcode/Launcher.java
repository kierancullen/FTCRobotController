package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.max;

public class Launcher {

    private static double tiltServoUp = 0.12;
    private static double tiltServoDown = 0;
    private static double pushServoIn = 0.28; // TODO update
    private static double pushServoOut = 0.215; // TODO update
    public static long strokeTime = 100; // In one direction (in/out time assumed same) // TODO update

    private static long turretStallThreshold = 100;
    private static double turretHomePower = -0.05; // TODO fill in
    private static double turretDrivePower = 0.1; // TODO fill in
    private static double turretHomeAngle = Math.toRadians(45); // relative to robot, zero is right TODO fill in
    private static double turretMaxAngle = Math.toRadians(135); // TODO fill in
    private static long turretCountsPerRadian = 0; // TODO fill in

    private double turretCurrentAngle;
    private long turretStallCounter;
    private long turretLastPosition;

    private BangBangFlywheel flywheel;
    private Servo pushServo;
    private DcMotor turret;

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public int disksRemaining;

    enum state {
        load,
        launchReady,
        launchPush,
        launchRetract
    }

    public Launcher(DcMotor flywheelMotor, Servo pushServo, DcMotor turret) {

        this.flywheel = new BangBangFlywheel(flywheelMotor, 28);
        this.pushServo = pushServo;
        this.turret = turret;
    }

    //Call this once to get everything set up
    public void initialize() {
        currentState = Launcher.state.load;
        lastState = Launcher.state.load;
        timeAtStateStart = System.currentTimeMillis();

        disksRemaining = 0;
    }

    //Call this in opmode init_loop
    public void home() {
        if (turretStallCounter > turretStallThreshold) {
            if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turret.setPower(0);
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            turretCurrentAngle = turretHomeAngle;
        }
        else {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setPower(turretHomePower);
        }

        // Update stall counter
        long turretCurrentPosition = turret.getCurrentPosition();
        if (turretCurrentPosition == turretLastPosition) {
            turretStallCounter++;
        }
        turretLastPosition = turretCurrentPosition;
    }

    // Set target angle relative to robot (zero is to the right).
    public void setTurretAngle(double targetAngleRads) {
        targetAngleRads = Math.max(targetAngleRads, turretHomeAngle); // right bound
        targetAngleRads = Math.min(targetAngleRads, turretMaxAngle); // left bound
        double angleFromHome = targetAngleRads - turretHomeAngle;
        long targetCounts = Math.round(turretCountsPerRadian * angleFromHome);
        turret.setTargetPosition((int)targetCounts);
        turret.setPower(turretDrivePower);
    }

    //Call this in the opmode loop
    boolean doingSingleLaunch = false;
    public void update(double launchRPM, boolean prepareLaunch, boolean goLaunch, boolean abort, boolean singleLaunch) {
        flywheel.update();
        if (currentState == state.load) {
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.stop();
            if (prepareLaunch) {
                currentState = state.launchReady;
                disksRemaining = 3;
            }
        }

        else if (currentState == state.launchReady) {
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (goLaunch && flywheel.currentState == BangBangFlywheel.state.running) {
                currentState = state.launchPush;
                doingSingleLaunch = false;
            }
            else if (singleLaunch && flywheel.currentState == BangBangFlywheel.state.running) {
                currentState = state.launchPush;
                disksRemaining = 1;
                doingSingleLaunch = true;
            }
            else if (abort) {
                currentState = state.load;
            }
        }

        else if (currentState == state.launchPush) {
            pushServo.setPosition(pushServoIn);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > strokeTime) {
                currentState = state.launchRetract;
                disksRemaining -= 1;
            }
            else if (abort) {
                currentState = state.load;
            }
        }
        else if (currentState == state.launchRetract) {
            pushServo.setPosition(pushServoOut);
            flywheel.setTargetRPM(launchRPM);
            flywheel.start();
            if (timeElapsedInState() > strokeTime && disksRemaining != 0) {
                currentState = state.launchPush;
            }
            else if (timeElapsedInState() > strokeTime && disksRemaining == 0) {
                if (doingSingleLaunch) currentState = state.launchReady;
                else currentState = state.load;
            }
            else if (abort) {
                currentState = state.load;
            }
        }

        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }
}

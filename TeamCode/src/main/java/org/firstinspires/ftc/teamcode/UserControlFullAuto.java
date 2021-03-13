package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="UserControlFullAuto")
public class UserControlFullAuto extends BaseOpmode{

    Point currentTarget;
    double currentAngle;
    Point baseTarget = new Point (122.994, 168.968);
    double baseAngle = Math.toRadians(99.2489);

    double launchRPM;
    final double powershotRPM = 3350;
    final double normalRPM = 4000;
    Point robotStartPosition;

    float storedPositionx;
    float storedPositiony;
    float storedAngle;

    enum state {
        starting,
        navigating,
        floating,
        disabled,
        launching,
        powershot
    }

    public state currentState;
    public state lastState;
    private long timeAtStateStart;

    public void init() {
        super.init();
        currentState = state.disabled;
        lastState = state.disabled;
        timeAtStateStart = System.currentTimeMillis();

        currentTarget = new Point (baseTarget.x, baseTarget.y);
        currentAngle = baseAngle;
        launchRPM = normalRPM;

        if(
        storage.read("robotPositionx") == 0 &&
        storage.read("robotPositiony") == 0 &&
        storage.read("robotAngle") == 0) {
            //For testing purposes
            telemetry.addLine("Auto was not run");
            storedPositionx =  (float)baseTarget.x;
            storedPositiony = (float)baseTarget.y;
            storedAngle = (float)baseAngle;
        }
        else {
            storedPositionx =  storage.read("robotPositionx");
            storedPositiony = storage.read("robotPositiony");
            storedAngle = storage.read("robotAngle");
            telemetry.addData("Loaded position from auto:", "x:" + storedPositionx + " y:" + storedPositiony + " angle:" + storedAngle);
        }


    }

    public void start() {
        super.start();
        intake.currentState = Intake.state.running;
        localizer.setPosition(new Point (storedPositionx, storedPositiony), storedAngle);
    }

    boolean goLaunch;
    boolean done;
    boolean powerShot;

    boolean prepareLaunch, triggerLaunch, abortLaunch, triggerSingleLaunch;

    public void loop() {
        telemetry.addData("Current state:", currentState);
        telemetry.addData("Launcher state:", launcher.currentState);
        telemetry.addData("Wobble state:", wobble.currentState);
        telemetry.addData("Wobble grabbing?:", wobble.grabbingNow);
        super.loop();
        goLaunch = gamepad1.x;
        done = gamepad1.a;
        powerShot = gamepad1.y;

        if (wobble.currentState != Wobble.state.stowed) intake.deflectorsStowed = true;
        else {intake.deflectors = true; intake.deflectorsStowed = false;}

        if (currentState == state.navigating) {
            //Tell our follower to navigate to the target
            follower.goToWaypoint(new Waypoint(currentTarget, currentAngle, Math.toRadians(180), 1.0, 1.0, 50, Math.toRadians(50)), true);
            if (!gamepadAllZero(gamepad1) && timeElapsedInState() > 500) { //If we start using the gamepad, go to the floating state
                currentState = state.floating;
            }
            else if (goLaunch && timeElapsedInState() > 500) {
                currentTarget = new Point(localizer.robotPosition.x, localizer.robotPosition.y);
                currentAngle = localizer.robotAngle;
                currentState = state.launching;
                drivetrain.allVelocitiesZero();
                triggerLaunch = true;
            }
            else if (done && timeElapsedInState() > 500) { //If we turn off navigation, go to the disabled state
                currentState = state.disabled;
                drivetrain.setBrake(true);
            }
        }

        else if (currentState == state.floating) {
            drivetrain.setVelocityFromGamepad(gamepad1); //Allow gamepad control now
            if (goLaunch && timeElapsedInState() > 500) {
                currentTarget = new Point(localizer.robotPosition.x, localizer.robotPosition.y);
                currentAngle = localizer.robotAngle;
                currentState = state.launching;
                drivetrain.allVelocitiesZero();
                triggerLaunch = true;
            }
            else if (done && timeElapsedInState() > 500) {
                currentState = state.disabled;
                intake.currentState = Intake.state.running;
            }
        }

        else if (currentState == state.disabled) {
            drivetrain.setVelocityFromGamepad(gamepad1);
            if (done && timeElapsedInState() > 500) {
                currentState = state.navigating;
                drivetrain.setBrake(true);
                follower.initialize();
                prepareLaunch = true;
                intake.currentState = Intake.state.stopped;
            }

            if (powerShot) {
                currentState = state.powershot;
                prepareLaunch = true;
                intake.currentState = Intake.state.stopped;
                launchRPM = powershotRPM;
                drivetrain.setBrake(true);
            }


        }

        else if (currentState == state.powershot) {
            intake.deflectorsStowed = true;
            drivetrain.setVelocityFromGamepad(gamepad1);
            if (timeElapsedInState() > 500 && powerShot) {
                triggerSingleLaunch = true;
            }
            if (done) {
                drivetrain.setBrake(true);
                currentState = state.disabled;
                intake.currentState = Intake.state.running;
                abortLaunch = true;
                launchRPM = normalRPM;
            }

        }

        else if (currentState == state.launching) {
            if (done && timeElapsedInState() > 500) {
                drivetrain.setBrake(true);
                currentState = state.disabled;
                intake.currentState = Intake.state.running;
                abortLaunch = true;
            }
            else if (launcher.disksRemaining == 0) {
                drivetrain.setBrake(true);
                currentState = state.disabled;
                intake.currentState = Intake.state.running;
            }
        }

        intake.update(false, gamepad1.back);
        launcher.update(launchRPM, prepareLaunch, triggerLaunch, abortLaunch, triggerSingleLaunch);
        wobble.update(gamepad1.dpad_left, gamepad1.dpad_up); //Wobble control is allowed anytime


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }
        lastState = currentState;

        //Automatically turn off our state machine triggers after we've already updated the state machines
        if (prepareLaunch) prepareLaunch = !prepareLaunch;
        if (triggerLaunch) triggerLaunch = !triggerLaunch;
        if (triggerSingleLaunch) triggerSingleLaunch = !triggerSingleLaunch;
        if (abortLaunch) abortLaunch = !abortLaunch;
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }

    private boolean gamepadAllZero(Gamepad gamepad) { //Checks if any of our drive controls are being pressed
        return Math.abs(gamepad.left_stick_y) == 0 && Math.abs(gamepad.right_stick_y) == 0
                && Math.abs(gamepad.right_trigger) == 0 && Math.abs(gamepad.left_trigger) == 0 && !gamepad1.left_bumper && !gamepad1.right_bumper;
    }


}

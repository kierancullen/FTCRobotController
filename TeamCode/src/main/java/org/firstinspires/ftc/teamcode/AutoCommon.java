package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.CVLinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.RingsCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Vision.RingsCV.SkystoneDeterminationPipeline.RingPosition.*;

@Autonomous(name = "AutoCommon")
public class AutoCommon extends BaseOpmode {

    OpenCvCamera webcam;
    CVLinearOpMode pipeline;

    enum state {
        starting,
        driveout1, driveout2,
        pause1, powershot1,
        strafe1, pause2, powershot2,
        strafe2, pause3, powershot3,
        ringposition,
        ringcollect,
        ringcollectreverse,
        aimprepare, aim, launchextras,
        ringposition2, ringcollect2, ringcollectreverse2,
        aimprepare2, aim2, launchextras2,
        drivetodropzone, dropdelay,
        driveback, pause4, grabwobble,
        drivetodropzone2, dropdelay2, dropwobble2,
        park
    }

    final double normalRPM = 4100;
    final double powershotRPM = 3250;

    Point robotStartPosition = new Point (121.92,21.955);
    Point noahOrigin = new Point (121.92, 21.955); //The point that all of Noah's points are relative to

    Waypoint driveout1 = new Waypoint(transform(10, 100, noahOrigin), Math.toRadians(90), Math.toRadians(0),
            0.75, 0.5, 100, Math.toRadians(40));
    Waypoint driveout2 = new Waypoint(transform(31.48, 152.4, noahOrigin), Math.toRadians(90), Math.toRadians(0),
            0.75, 0.5, 20, Math.toRadians(40));
    Waypoint strafe1 = new Waypoint(transform(31.48, 152.4, noahOrigin), Math.toRadians(96), Math.toRadians(90),
            0.5, 0.7, 20, Math.toRadians(40));
    Waypoint strafe2 = new Waypoint(transform(31.48, 152.4, noahOrigin), Math.toRadians(85), Math.toRadians(90),
            0.5, 0.7, 20, Math.toRadians(40));
    Waypoint ringposition = new Waypoint(transform(10, 103, noahOrigin), Math.toRadians(180), Math.toRadians(-90),
            1.0, 1.0, 50, Math.toRadians(40));
    Waypoint aim = new Waypoint(transform(-30.48, 152.4, noahOrigin), Math.toRadians(90), Math.toRadians(0),
            1.0, 0.7, 60, Math.toRadians(40));
    Waypoint dropzone1;
    Waypoint dropzone2;
    Waypoint goalgrabpoint = new Waypoint(transform(-28, 52.95, noahOrigin), Math.toRadians(90), Math.toRadians(180),
            0.9, 0.7, 70, Math.toRadians(40));
    Waypoint goalstrafepoint = new Waypoint(transform(-36, 52.95, noahOrigin), Math.toRadians(90), Math.toRadians(180),
            0.9, 0.7, 10, Math.toRadians(40));
    Waypoint park = new Waypoint(transform(0, 182.88, noahOrigin), Math.toRadians(90), Math.toRadians(180),
            0.9, 0.7, 70, Math.toRadians(40));

    public state currentState;
    public state lastState;
    public RingsCV.SkystoneDeterminationPipeline.RingPosition ringCondition;
    private long timeAtStateStart;

    public void init() {
        super.init();
        intake.gatePushing = true;
        drivetrain.setBrake(true);
        currentState = state.starting;
        lastState = state.starting;
        timeAtStateStart = System.currentTimeMillis();
        wobble.currentState = Wobble.state.matchStartOpen;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new CVLinearOpMode();
        webcam.setPipeline(pipeline);

        //These positions will be stored if auto does not complete
        storage.write("robotPositionx", (float)(0.0));
        storage.write("robotPositiony", (float)(0.0));
        storage.write("robotAngle", (float)(0.0));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });


    }

    public void init_loop(){
        ringCondition = pipeline.position;
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        if (gamepad1.a) wobble.currentState = Wobble.state.matchStartClosed;
        wobble.update(false, false); //We need to do this so that we can load the wobble goal and have it grab it during init
    }

    public void start() {
        super.start();
        follower.initialize();
        localizer.setPosition(robotStartPosition, Math.toRadians(90));

        //Determine where we're going to drop the wobble goal
        if (ringCondition == FOUR) {
            dropzone1 = new Waypoint(transform(-55.96, 290.325, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
            dropzone2 = new Waypoint(transform(-40.96, 290.325, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
        }

        else if (ringCondition == ONE) {
            dropzone1 = new Waypoint(transform(-3, 275.365, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
            dropzone2 = new Waypoint(transform(-3, 245.365, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
        }

        else {
            dropzone1 = new Waypoint(transform(-63, 214.504, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
            dropzone2 = new Waypoint(transform(-63, 175.396, noahOrigin), Math.toRadians(90), Math.toRadians(0),
                    0.9, 0.7, 45, Math.toRadians(40));
        }
    }


    boolean prepareLaunch = false;
    boolean goLaunch = false;
    boolean abortLaunch = false;
    boolean singleLaunch = false;

    boolean wobblePrepare = false;
    boolean wobbleGrab = false;

    double launchRPM;

    double launch1Angle = 0;
    double launch2Angle = 0;
    double launch3Angle = 0;

    public void loop() {
        telemetry.addData("launch 1 angle:", launch1Angle);
        telemetry.addData("launch 2 angle:", launch2Angle);
        telemetry.addData("launch 3 angle:", launch3Angle);
        telemetry.addData("Intake ring count:", intake.ringCount);
        telemetry.addData("Follower x:", follower.movementXState);
        telemetry.addData("Follower y:", follower.movementYState);
        telemetry.addData("Follower turn:", follower.turningState);
        telemetry.addData("Auto state:", currentState);
        telemetry.addData("Path state:", follower.overallState);
        telemetry.addData("Launcher state", launcher.currentState);
        super.loop();

        if (currentState == state.starting) {
            launchRPM = powershotRPM;
            wobble.grabbingNow = true;
            prepareLaunch = true;
            if (timeElapsedInState() > 500) {
                follower.initialize();
                currentState = state.driveout1;
            }
        }
        else if (currentState == state.driveout1) {
            currentState = state.driveout2;
        }

        else if (currentState == state.driveout2) {
            follower.goToWaypoint(driveout2, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause1;
                prepareLaunch = false;
            }
        }

        else if (currentState == state.pause1) {
            follower.goToWaypoint(driveout2, true);
            follower.pidTurn(driveout2.staticAngle);
            if (Math.abs(localizer.robotAngle - (driveout2.staticAngle)) < Math.toRadians(0.2)
                    && timeElapsedInState() > 200) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot1;
            }
        }


        else if (currentState == state.powershot1) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                launch1Angle = Math.toDegrees(localizer.robotAngle);
                follower.initialize();
                currentState = state.strafe1;
            }
        }

        else if (currentState == state.strafe1) {
            follower.goToWaypoint(strafe1, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause2;
            }
        }

        else if (currentState == state.pause2) {
            follower.goToWaypoint(strafe1, true);
            follower.pidTurn(strafe1.staticAngle);
            if (Math.abs(localizer.robotAngle - (strafe1.staticAngle)) < Math.toRadians(0.2)
                    && timeElapsedInState() > 200) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot2;
            }
        }

        else if (currentState == state.powershot2) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                launch2Angle = Math.toDegrees(localizer.robotAngle);
                follower.initialize();
                currentState = state.strafe2;
            }
        }

        else if (currentState == state.strafe2) {
            follower.goToWaypoint(strafe2, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.pause3;
            }
        }

        else if (currentState == state.pause3) {
            follower.goToWaypoint(strafe2, true);
            follower.pidTurn(strafe2.staticAngle);
            if (Math.abs(localizer.robotAngle - (strafe2.staticAngle)) < Math.toRadians(0.2)
                    && timeElapsedInState() > 200) {
                drivetrain.allVelocitiesZero();
                follower.initialize();
                singleLaunch = true;
                currentState = state.powershot3;
            }
        }

        else if (currentState == state.powershot3) {
            singleLaunch = false;
            if (timeElapsedInState() > launcher.strokeTime * 2) {
                launch3Angle = Math.toDegrees(localizer.robotAngle);
                follower.initialize();
                currentState = state.ringposition;
                abortLaunch = true;
            }
        }

        else if (currentState == state.ringposition) {
            follower.goToWaypoint(ringposition, true);
            launchRPM = normalRPM;
            if (follower.overallState == Follower.pathState.passed) {
                if (ringCondition == FOUR || ringCondition == ONE) {
                    currentState = state.ringcollect;
                    intake.currentState = Intake.state.running;
                }
                else {
                    currentState = state.drivetodropzone;
                    follower.initialize();
                    drivetrain.allVelocitiesZero();
                    intake.gate = false;
                    wobble.currentState = Wobble.state.floatingAuto;
                }
            }
        }

        else if (currentState == state.ringcollect) {
            //Drive forward slowly until we think we've gotten three
            drivetrain.translateVelocity.x = 0;
            drivetrain.translateVelocity.y = 0.2;
            drivetrain.turnVelocity = 0;
            if (timeElapsedInState() > 1250) {
                currentState = state.ringcollectreverse;
            }
        }

        else if (currentState == state.ringcollectreverse) {
            drivetrain.translateVelocity.x = 0;
            drivetrain.translateVelocity.y = -1;
            drivetrain.turnVelocity = 0;
            if (timeElapsedInState() > 300) {
                currentState = state.aim;
                drivetrain.allVelocitiesZero();
                follower.initialize();
                abortLaunch = false;
            }
        }

        //Start doing the aim move before preparing the launcher so that the intake has a little more time to transfer the last ring
        else if (currentState == state.aim) {
            follower.goToWaypoint(aim, true);
            if (timeElapsedInState() > 1000) {
                currentState = state.aimprepare;
            }

        }

        //Now prepare the launcher
        else if (currentState == state.aimprepare) {
            prepareLaunch = true;
            follower.goToWaypoint(aim, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.launchextras;
                intake.currentState = Intake.state.stopped;
                goLaunch = true;
            }
        }

        else if (currentState == state.launchextras) {
            follower.goToWaypoint(aim, true);
            if (launcher.disksRemaining == 1) {
                if (ringCondition == FOUR) {
                    currentState = state.ringposition2;
                    prepareLaunch = false;
                    goLaunch = false;
                    abortLaunch = true;
                    drivetrain.allVelocitiesZero();
                    follower.initialize();
                }
                else if (ringCondition == ONE) {
                    currentState = state.drivetodropzone;
                    prepareLaunch = false;
                    goLaunch = false;
                    abortLaunch = true;
                    drivetrain.allVelocitiesZero();
                    follower.initialize();
                    wobble.currentState = Wobble.state.floatingAuto;
                }



            }
        }

        else if (currentState == state.ringposition2) {
            follower.goToWaypoint(ringposition, true);
            launchRPM = normalRPM;
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.ringcollect2;
                intake.currentState = Intake.state.running;
            }
        }

        else if (currentState == state.ringcollect2) {
            //Drive forward slowly until we think we've gotten three
            drivetrain.translateVelocity.x = 0;
            drivetrain.translateVelocity.y = 0.2;
            drivetrain.turnVelocity = 0;
            if (timeElapsedInState() > 2500) {
                currentState = state.ringcollectreverse2;
            }
        }

        else if (currentState == state.ringcollectreverse2) {
            drivetrain.translateVelocity.x = 0;
            drivetrain.translateVelocity.y = -1;
            drivetrain.turnVelocity = 0;
            if (timeElapsedInState() > 300) {
                currentState = state.aim2;
                drivetrain.allVelocitiesZero();
                follower.initialize();
                abortLaunch = false;
                prepareLaunch = true;
            }
        }

        else if (currentState == state.aim2) {
            follower.goToWaypoint(aim, true);
            if (timeElapsedInState() > 1000) {
                currentState = state.aimprepare2;
            }
        }

        else if (currentState == state.aimprepare2) {
            prepareLaunch = true;
            follower.goToWaypoint(aim, true);
            if (follower.overallState == Follower.pathState.passed) {
                currentState = state.launchextras2;
                intake.currentState = Intake.state.stopped;
                goLaunch = true;
            }
        }

        else if (currentState == state.launchextras2) {
            follower.goToWaypoint(aim, true);
            if (launcher.disksRemaining == 1) {
                currentState = state.drivetodropzone;
                prepareLaunch = false;
                goLaunch = false;
                abortLaunch = true;

                follower.initialize();
                drivetrain.allVelocitiesZero();
                wobble.currentState = Wobble.state.floatingAuto;
                intake.gate = false;
            }
        }

        else if (currentState == state.drivetodropzone) {
            follower.goToWaypoint(dropzone1, false);
            if (follower.distanceTo(dropzone1.location) < 25) {
                wobble.grabbingNow=false; //The servo takes time to open, so start a little early
            }
            if (follower.overallState == Follower.pathState.passed) {
                if (ringCondition == FOUR) {
                    wobble.grabbingNow=false;
                    wobble.currentState = Wobble.state.stowed;
                    currentState = state.driveback;

                    follower.initialize();
                    drivetrain.allVelocitiesZero();
                }
                else {
                    wobble.grabbingNow=false;
                    wobble.currentState = Wobble.state.stowed;
                    currentState = state.dropdelay;

                    follower.initialize();
                    drivetrain.allVelocitiesZero();
                }

            }

        }

        //For the closer two boxes, we can't just release the wobble while moving because there isn't enough time in the move
        else if (currentState == state.dropdelay) {
            //We just wait a bit
            if (timeElapsedInState() > 1000) {
                currentState = state.driveback;
            }
        }

        else if (currentState == state.driveback) {
            follower.goToWaypoint(goalgrabpoint, true);
            if ((localizer.robotAngle - Math.toRadians(90)) < Math.toRadians(0.15)
                    && (follower.distanceTo(goalgrabpoint.location) < 0.5)) {
                wobble.currentState = Wobble.state.down;
                currentState = state.pause4;
            }
        }

        else if (currentState == state.pause4) {
            follower.goToWaypoint(goalstrafepoint, true);
            //Waiting for arm to drop
            if (timeElapsedInState() > 750) {
                currentState = state.grabwobble;
                follower.initialize();
                drivetrain.allVelocitiesZero();
            }
        }

        else if (currentState == state.grabwobble) {
            wobble.grabbingNow = true;
            if (timeElapsedInState() > 750) {
                currentState = state.drivetodropzone2;
            }
        }

        else if (currentState == state.drivetodropzone2) {
            follower.goToWaypoint(dropzone2, false);
            wobble.currentState = Wobble.state.floatingAuto;
            if (follower.distanceTo(dropzone2.location) < 25) {
                wobble.grabbingNow=false; //The servo takes time to open, so start a little early
            }
            if (follower.overallState == Follower.pathState.passed) {
                wobble.grabbingNow=false;
                wobble.currentState = Wobble.state.stowed;
                currentState = state.park;

                follower.initialize();
                drivetrain.allVelocitiesZero();
            }
        }

        else if (currentState == state.park) {
            follower.goToWaypoint(park, true);
        }


        if (currentState != lastState) {
            // There was a state transition
            timeAtStateStart = System.currentTimeMillis();
        }

        lastState = currentState;
        updateStateMachines();

    }

    public void stop() {
        //Store our current location for use in teleop
        storage.write("robotPositionx", (float)localizer.robotPosition.x);
        storage.write("robotPositiony", (float)localizer.robotPosition.y);
        storage.write("robotAngle", (float)localizer.robotAngle);
        telemetry.addData("Stored position from auto:", "x:" + localizer.robotPosition.x + " y:" + localizer.robotPosition.y + " angle:" + localizer.robotAngle);
    }

    private void updateStateMachines() {
        launcher.update(launchRPM, prepareLaunch, goLaunch, abortLaunch, singleLaunch);
        intake.update(false, false); //No variables for these because we're just switching states manually
        wobble.update(wobblePrepare, wobbleGrab);
    }

    private long timeElapsedInState() {
        return System.currentTimeMillis() - timeAtStateStart;
    }

    //Takes points that are relative to some other origin and changes them to be relative to the field origin (bottom left corner)
    public Point transform(double x, double y, Point relativeTo) {
        return new Point (relativeTo.x + x, relativeTo.y + y);
    }

}

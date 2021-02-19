package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class UserControl extends BaseOpmode {

    TeleOpPositioner positioner;

    final Point launchingTarget = new Point (213.16, 365.76);
    double LaunchRPM = 5000;

    public void init() {
        super.init();
        positioner = new TeleOpPositioner();
    }

    public void start() {
        super.start();
        localizer.setPosition(new Point(304.8,21.955),  Math.toRadians(90)); //whatever the start position is after auto, actually
        positioner.initialize();
    }

    boolean toggleNavigation = false;
    boolean xLast;
    public void loop() {

        if (!xLast && gamepad1.x) toggleNavigation = !toggleNavigation;
        xLast = gamepad1.x;

        telemetry.addData("Launch RPM:", LaunchRPM);
        telemetry.addData("Distance to target", follower.distanceTo(launchingTarget));

        super.loop();
        intake.update(gamepad1.right_bumper, gamepad1.left_bumper);
        launcher.update(LaunchRPM, gamepad1.dpad_up, gamepad1.y, gamepad1.a, gamepad1.b);
        positioner.update(drivetrain, localizer, follower, gamepad1, toggleNavigation, launchingTarget);
        /*telemetry.addData("Left raw", localizer.left.currentValue);
        telemetry.addData("Right raw", localizer.right.currentValue);
        telemetry.addData("Center raw", localizer.center.currentValue);
        telemetry.addData("Left odometer", localizer.left.totalDelta);
        telemetry.addData("Right odometer", localizer.right.totalDelta);
        telemetry.addData("Center odometer", localizer.center.totalDelta);
        telemetry.addData("Localizer x:", localizer.robotPosition.x);
        telemetry.addData("Localizer y:", localizer.robotPosition.y);
        telemetry.addData("Localizer angle", Math.toDegrees(localizer.robotAngle)); */

    }

    public double getLaunchRPM() {
        double distanceToTarget = follower.distanceTo(launchingTarget);
        return (-4.537 * distanceToTarget) + 5429.8;
    }

}

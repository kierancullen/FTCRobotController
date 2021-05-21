package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Legacy.TeleOpPositioner;

//Current class for TeleOp
@TeleOp(name="UserControl")
public class UserControl extends BaseOpmode {

    TeleOpPositioner positioner;

    final Point launchingTarget = new Point (213.16, 365.76); //Real coordinates of the front of the goal
    double LaunchRPM = 5000;

    public void init() {
        super.init();
        positioner = new TeleOpPositioner();
    }

    public void start() {
        super.start();
        localizer.setPosition(new Point(0,0),  Math.toRadians(90)); //this would be whatever the start position is after auto, actually
        positioner.initialize();
    }

    boolean toggleNavigation = false;
    boolean xLast;
    public void loop() {

        //Currently just some code for tweaking the launcher power
        if (!xLast && gamepad1.x) toggleNavigation = !toggleNavigation;
        xLast = gamepad1.x;
        if (gamepad1.right_stick_button) LaunchRPM+= 10;
        if (gamepad1.left_stick_button) LaunchRPM-= 10;

        telemetry.addData("Drivetrain x", drivetrain.translateVelocity.x);
        telemetry.addData("Launch RPM:", LaunchRPM);
        telemetry.addData("Distance to target", follower.distanceTo(launchingTarget));
        telemetry.addData("Rings:", intake.ringCount);

        super.loop();
        intake.update(gamepad1.right_bumper, gamepad1.left_bumper);
        launcher.update(LaunchRPM, gamepad1.dpad_up, gamepad1.y, gamepad1.a, gamepad1.b);
        positioner.update(drivetrain, localizer, follower, gamepad1, toggleNavigation, launchingTarget);

    }

    //Function to get the Launch RPM based on distance

    public double getLaunchRPM() {
        double distanceToTarget = follower.distanceTo(launchingTarget);
        if (distanceToTarget > 335.28) {
            return (8.206 * distanceToTarget) + 978.58;
        }
        else if (distanceToTarget < 335.28 && distanceToTarget > 200.84) {
            return (-0.984 * distanceToTarget) + 4060;
        }
        else {
            return (-16.73 * distanceToTarget) + 7900;
        }
    }

}

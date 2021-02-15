package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="UserControl")
public class UserControl extends BaseOpmode {


    public void init() {
        super.init();
        drivetrain.setBrake(false);
    }

    public void start() {
        super.start();
    }

    public void loop() {
        super.loop();
        telemetry.addData("Left raw", localizer.left.currentValue);
        telemetry.addData("Right raw", localizer.right.currentValue);
        telemetry.addData("Center raw", localizer.center.currentValue);
        telemetry.addData("Left odometer", localizer.left.totalDelta);
        telemetry.addData("Right odometer", localizer.right.totalDelta);
        telemetry.addData("Center odometer", localizer.center.totalDelta);
        telemetry.addData("Localizer x:", localizer.robotPosition.x);
        telemetry.addData("Localizer y:", localizer.robotPosition.y);
        telemetry.addData("Localizer angle", Math.toDegrees(localizer.robotAngle));
        telemetry.addData("Turn scaling factor", localizer.turnScalingFactor);
    }

}

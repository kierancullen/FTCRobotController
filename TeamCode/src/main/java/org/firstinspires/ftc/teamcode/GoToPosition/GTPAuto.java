package org.firstinspires.ftc.teamcode.GoToPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NoahOdometry.OdometryCalculations;
import org.firstinspires.ftc.teamcode.NoahOdometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.GoToPosition.GTPCalculations.changeInError;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPCalculations.d;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPCalculations.dIsNotZero;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPCalculations.dIsZero;
import static org.firstinspires.ftc.teamcode.NoahOdometry.OdometryCalculations.coordinatePositionUpdate;


@Autonomous(name = "OdometryLinOpMode", group = "Autonomous")
public class GTPAuto extends LinearOpMode {
    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;


    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "left back", verticalRightEncoderName = "left front", horizontalEncoderName = "right back";

    static OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    static OdometryCalculations odometryCalculations;
    Thread updateOdometry;

    final double COUNTS_PER_INCH = 1141.94659527;
    public static double globalHeading, globalXPosEncoderTicks, globalYPosEncoderTicks;

    public static boolean finalPoint = false;
    static double[] powers;
    static double pidOutput;
    public static double verticalLeftPosition, verticalRightPosition, horizontalPosition;

    public static double bestAngle = 1000;

    public static int coordinateNumber = 0;

    public static double lastPoint = 0;

    private double[]testX = {0,25,25,0};
    private double[]testY = {0,25,25,50};
    private double[]testHeading = {bestAngle,bestAngle,bestAngle,0};

    private double[] xCoordinates1 = {15,8,0,-10};
    private double [] yCoordinates1 = {36,50,53,56};
    private double [] headings1 = {bestAngle,bestAngle,bestAngle,0};

    private double [] xCoordinates2 = {-12};
    private double [] yCoordinates2 = {61};
    private double [] headings2 = {180};

    private double [] xCoordinates3 = {-12,-12,-12,-12};
    private double [] yCoordinates3 = {48, 36, 61,61};
    private double [] headings3 = {180,180,180,0};

    public void runOpMode() {



            // goToPosition = new MotorPowerMecanum();
            // pid = new PIDCalulations();



            leftFrontWheel = hardwareMap.dcMotor.get("left front");
            leftBackWheel = hardwareMap.dcMotor.get("left back");
            rightFrontWheel = hardwareMap.dcMotor.get("right front");
            rightBackWheel = hardwareMap.dcMotor.get("right back");
            verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
            verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
            horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

            //Reset the encoders
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeftPosition = verticalLeft.getCurrentPosition();
            verticalRightPosition = verticalRight.getCurrentPosition();
            horizontalPosition = horizontal.getCurrentPosition();

            /*
            Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
            such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
            horizontal encoder travels to the right, it returns positive value
            */
           // verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
           // verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);

            //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Init complete
            telemetry.addData("Status", "Init Complete");
            telemetry.update();

            rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

            coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition);

            waitForStart();

        if (opModeIsActive()) {
            /*heading = globalPositionUpdate.robotOrientationRadians;
            globalXPosEncoderTicks = globalPositionUpdate.returnXCoordinate();
            globalYPosEncoderTicks = globalPositionUpdate.returnYCoordinate();*/

            /*go(xCoordinates1, yCoordinates1, headings1);
            sleep(2000);
            go(xCoordinates2, yCoordinates2, headings2);
            sleep(400);
            go(xCoordinates3, yCoordinates3, headings3);*/




            /*grabFoundation();
            sleep(800);
            go(xCoordinates3i, yCoordinates3i, headings3i);
            releaseFoundation();
            sleep(200);
            intakeOn();
            go(xCoordinates4i, yCoordinates4i, headings4i);*/

            go(testX,testY,testHeading);

        }

    }



    // set power to each motor
    public void setPower(double lf, double lb, double rf, double rb) {
        leftFrontWheel.setPower(lf/1.5);
       leftBackWheel.setPower(lb/1.5);
       rightFrontWheel.setPower(rf/1.5);
       rightBackWheel.setPower(rb/1.5);
    }

    public void go(double[] x, double[] y, double[] heading) {
        do {
            // update global positions
            verticalLeftPosition = verticalLeft.getCurrentPosition();
            verticalRightPosition = verticalRight.getCurrentPosition();
            horizontalPosition = horizontal.getCurrentPosition();


            globalXPosEncoderTicks = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[0];
            globalYPosEncoderTicks = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[1];
            globalHeading = coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition)[2];

            //Calling in method from calculations class
            coordinatePositionUpdate(verticalLeftPosition, verticalRightPosition, horizontalPosition);

            //globalHeading = odometryCalculations.coordinatePositionUpdate()[2];
            //globalXPosEncoderTicks = odometryCalculations.coordinatePositionUpdate()[0];
            //globalYPosEncoderTicks = odometryCalculations.coordinatePositionUpdate()[1];

            // calculate powers and set them to the respective motors
            powers = GTPCalculations.goToPositionCalculations(x, y, heading);
            setPower(powers[0]*powers[5], powers[1]*powers[5], powers[2]*powers[5], powers[3]*powers[5]);
            telemetry.addData("final point", powers[6]);
            telemetry.addData("coordinateNumber", coordinateNumber);
            telemetry.addData("c", powers[4]);
            telemetry.addData("globalX", globalXPosEncoderTicks/COUNTS_PER_INCH);
            telemetry.addData("globalY", globalYPosEncoderTicks/COUNTS_PER_INCH);
            telemetry.addData("globalHeading", globalHeading);
            telemetry.addData("when D is 0", dIsZero);
            telemetry.addData("when D is NOT 0", dIsNotZero);
            telemetry.addData("d", d);
            telemetry.addData("changeInError", changeInError);
            telemetry.update();
        } while (opModeIsActive() & powers[6] < 15 || powers[4] > 1.2 || d > 0.001);

        // stop
        setPower(0, 0, 0, 0);
        coordinateNumber = 0;

    }


}
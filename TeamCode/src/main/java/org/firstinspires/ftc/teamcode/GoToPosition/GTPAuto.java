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

    /*Here is where I put the coordinates for the points. I will group by which coordinates they are (the 1st set, second set,
    etc.) Following the name, there will be both a number and possibly a letter. The number refers to which set of coordinates
    they are. The letter refers to for which autonomous case, as identified by the CV (A, B, or C), each will be used for.*/

    //GOING TO 1ST POWERSHOT TARGET
    private double[] xCoordinates1 = {0, 3, 3};
    private double[] yCoordinates1 = {0, 36, 36};
    private double[] headings1 = {bestAngle, bestAngle, 0};

    //GOING TO 2ND POWERSHOT TARGET
    private double[] xCoordinate2 = {9.5};
    private double[] yCoordinate2 = {36};
    private double[] heading2 = {0};

    //GOING TO 3RD POWERSHOT TARGET
    private double[] xCoordinate3 = {16};
    private double[] yCoordinate3 = {36};
    private double[] heading3 = {0};

    //GOING TO COLLECT RINGS
    private double[] xCoordinates4 = {0,-10};
    private double[] yCoordinates4 = {37,37};
    private double[] headings4 = {bestAngle,-90};

    //SHOOTING RINGS
    private double[] xCoordinate5 = {-9};
    private double[] yCoordinate5 = {40};
    private double[] heading5 = {0};

    //GOING TO DEPOSIT 1ST WOBBLE GOAL
    private double[] xCoordinate6 = {3};
    private double[] yCoordinate6 = {85};
    private double[] heading6 = {bestAngle};

    //GOING TO PICK UP 2ND WOBBLE GOAL
    private double[] xCoordinates7 = {-20,-28};
    private double[] yCoordinates7 = {75,30};
    private double[] headings7 = {-90,-90};

    //GOING TO DEPOSIT 2ND WOBBLE GOAL
    private double[] xCoordinate8 = {-3,0};
    private double[] yCoordinate8 = {30, 85};
    private double[] heading8 = {0,0};

    //PARKING ON MIDLINE
    private double[] xCoordinate9 = {-10};
    private double[] yCoordinate9 = {58};
    private double[] heading9 = {0};

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



            go(xCoordinates1,yCoordinates1,headings1);
            sleep(1000);
            //SHOOT 1ST POWERSHOT TARGET
            go(xCoordinate2,yCoordinate2,heading2);
            sleep(1000);
            //SHOOT 2ND POWERSHOT TARGET
            go(xCoordinate3,yCoordinate3,heading3);
            sleep(1000);
            //SHOOT 3RD POWERSHOT TARGET
            //TURN ON INTAKE
            go(xCoordinates4,yCoordinates4,headings4);
            //TURN OFF INTAKE
            go(xCoordinate5,yCoordinate5,heading5);
            sleep(2000);
            //SHOOT RINGS
            go(xCoordinate6,yCoordinate6,heading6);
            sleep(1500);
            //LET GO OF WOBBLE GOAL
            go(xCoordinates7,yCoordinates7,headings7);
            sleep(2000);
            //GRAB WOBBLE GOAL
            go(xCoordinate8,yCoordinate8,heading8);
            sleep(1500);
            //LET GO OF WOBBLE GOAL
            go(xCoordinate9,yCoordinate9,heading9);

            //go(testX,testY,testHeading);

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
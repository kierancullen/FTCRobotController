package org.firstinspires.ftc.teamcode.GoToPosition;

import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.GoToPosition.GTPAuto.coordinateNumber;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPAuto.globalHeading;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPAuto.globalXPosEncoderTicks;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPAuto.globalYPosEncoderTicks;
import static org.firstinspires.ftc.teamcode.GoToPosition.GTPAuto.lastPoint;

public class GTPCalculations {
    public static double Theta, c, linearDistance, rotationalDistance;
    public static double outputX;
    public static double outputY, headingForTurning;
    public static double reductionDistance;
    public static double distanceToTurn;
    public static double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, powerReduction, changeInError, currentError;
    public static double circumference = 64.42;
    public static double dIsZero = 0;
    public static double dIsNotZero = 0;
    public static double lastPointisTrue;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final static double COUNTS_PER_INCH = 1141.94659527;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    public static double globalXPos, globalYPos, proportionPowerReduction, turnPower;
    public static double xPowerRatio, yPowerRatio;

    public static double previousError, previousTime, currentTime, p, i, d, pidOutput, previousError2 = 0, previousError3 = 0,
            previousError4 = 0, previousError5 = 0, previousError6 = 0, previousError7 = 0;
    public static double prevD = 0;
    public static int arraySize;
    public double decreasePower;



    public static double[] goToPositionCalculations (double[] desiredXCoordinate, double[] desiredYCoordinate, double[] desiredHeading) {
        //Converting the odometry readings in encoder ticks to inches
        globalXPos = globalXPosEncoderTicks / COUNTS_PER_INCH;
        globalYPos = globalYPosEncoderTicks / COUNTS_PER_INCH;

        //Getting the ratio of motor powers based off the distance to target in each axis
        xPowerRatio = (desiredXCoordinate[coordinateNumber] - globalXPos);
        yPowerRatio = (desiredYCoordinate[coordinateNumber] - globalYPos);

        //Finding the reduction factor based off the distance to target
        /*reductionDistance = Range.clip(c, 0, 25);
        proportionPowerReduction = Range.clip(Math.sqrt(Math.abs(c / 25)), 0, 1);*/

        if (globalHeading >= 0) {
            headingForTurning = globalHeading;
        }
        if (globalHeading < 0) {
            headingForTurning = globalHeading + Math.PI;
        }

        arraySize = desiredHeading.length - 1;

        if (c < 3 && coordinateNumber != arraySize) {
            coordinateNumber += 1;
        } if (coordinateNumber > arraySize) {
            coordinateNumber = arraySize;
        } if (coordinateNumber != arraySize) {
            lastPoint = 0;
        } else {
            lastPoint += 1;
        }


        if(desiredHeading[coordinateNumber] != 1000) {
            distanceToTurn = desiredHeading[coordinateNumber] - Math.toDegrees(globalHeading);
        }  else if (c > 6 && desiredHeading[coordinateNumber] == 1000){
            distanceToTurn = Math.toDegrees(Math.atan2(xPowerRatio,yPowerRatio)) - Math.toDegrees(globalHeading);
        } else {
            distanceToTurn = 0;
        }
        turnPower = distanceToTurn / 360 * circumference;
        return driveMecanum(xPowerRatio, yPowerRatio, turnPower, 0, arraySize, coordinateNumber, lastPoint);


    }

    public static double[] driveMecanum(double xPower, double yPower, double turnPower, double reduction, double array, double coordinate, double finalPoint) {
        rotationalDistance = Math.abs((distanceToTurn / 360) * circumference);
        linearDistance = Math.sqrt(xPower * xPower + yPower * yPower);
        c = linearDistance + Math.abs((distanceToTurn / 360) * circumference);

        Theta = Math.atan2(xPower, yPower);

        outputY = Math.cos(globalHeading - Theta) * linearDistance;
        outputX = -Math.sin(globalHeading - Theta) * linearDistance;

        leftFrontPower = (outputY + outputX) + turnPower;
        leftBackPower = (outputY - outputX) + turnPower;
        rightFrontPower = (outputY - outputX) - turnPower;
        rightBackPower = (outputY + outputX) - turnPower;

        powerReduction = reduction;

        double[] wheelPowers = {Math.abs(rightFrontPower), Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)};
        Arrays.sort(wheelPowers);
        double biggestInput = wheelPowers[3];

        leftFrontPower /= biggestInput;
        leftBackPower /= biggestInput;
        rightFrontPower /= biggestInput;
        rightBackPower /= biggestInput;

        currentTime = System.currentTimeMillis();
        changeInError = c - previousError;

        p = c * 0.23;
        d = (changeInError / (currentTime - previousTime)) * 31;
        i = 0;


        if (coordinate != array) {
            pidOutput = 1;
        } else {
            pidOutput = Range.clip((p + i + d), -1, 1);
        }

        previousError = c;
        previousTime = currentTime;

        lastPointisTrue = finalPoint;

        return new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower, c, pidOutput, lastPointisTrue};
    }

}

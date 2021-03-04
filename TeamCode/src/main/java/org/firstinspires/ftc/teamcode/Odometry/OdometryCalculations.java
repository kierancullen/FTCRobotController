package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.horizontalOffset;
import static org.firstinspires.ftc.teamcode.Odometry.OdometryVariables.trackwidth;

public class OdometryCalculations {

    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;
    private static double verticalLeftEncoderWheelPosition, verticalRightEncoderWheelPosition, previousVerticalLeftEncoderWheelPosition, previousVerticalRightEncoderWheelPosition;
    private static double changeInRobotOrientation, headingRadians, normalEncoderWheelPosition,prevNormalEncoderWheelPosition;
    public static double COUNTS_PER_INCH = 1141.94659527;
    public static double robotEncoderWheelDistance = trackwidth*COUNTS_PER_INCH;
    public static double globalXPos, globalYPos;




    public static double[] coordinatePositionUpdate (double leftEncoderPosition, double rightEncoderPosition, double horizontalEncoderPosition){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = -(leftEncoderPosition);
        verticalRightEncoderWheelPosition = (rightEncoderPosition);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        headingRadians = ((headingRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoderPosition);
        //double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        //double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double rawHorizontalChange = (normalEncoderWheelPosition - prevNormalEncoderWheelPosition);
        double horizontalChange = rawHorizontalChange - changeInRobotOrientation*horizontalOffset*1141.94659527;

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        globalXPos = globalXPos + (p* Math.sin(headingRadians) + n* Math.cos(headingRadians));
        globalYPos = globalYPos - (p* Math.cos(headingRadians) - n* Math.sin(headingRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;



        return new  double[] {globalXPos, globalYPos, -headingRadians};
    }

}

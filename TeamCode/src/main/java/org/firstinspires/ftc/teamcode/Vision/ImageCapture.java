
package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

//Just used this thing from last year to grab some images because I don't know how to do it with OpenCV :)
@Autonomous(name = "ImageCapture")
public class ImageCapture extends LinearOpMode {

    public static int[][] STONE_LOCATIONS_BLUE = new int[][]{
            {168, 381},
            {245, 382},
            {354, 381},
    };


    public static int[][] STONE_LOCATIONS_RED = new int[][]{
            {821, 378},
            {728, 380},
            {611, 381},
    };

    private static final String VUFORIA_KEY =
            "Ac0A5xL/////AAABmbaZRuKrykmMhgpBAfm4wxkWMeMkHp/ij0Bv8cnqyigZaQN4qUU9wK+CmT4WDTRnZef/AEyluCOS1Z8a5pwiHeJpjLNqVcQoQsXBJT06NyKXZ2v2BDMqURXAnLCl82w+vIY3u4W/XdtFBt2m0/5OQNLFZRaIz3LJZaXGYz4hSRFAyMj0yVonukAXvjQljMxjd1YNUhpXk8V3qJaXS49Ep69t0AypLu+hE2AdHg1e15q29AifPAANhWM0PpWEACCVn7RWe19wyNi6N8Ab0c77kudZoGWmQF4hZVGRKK3ZrVz7kz1wyk3tfzHUsteJm7hbw8kagADt2ZKBDkO4+0i0HtB2hXcrKUp/w23nNTtY4SJ0";

    private VuforiaLocalizer vuforia;

    public void runOpMode() {

        initVuforia();

        waitForStart();

        sleep(5000);

        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(AppUtil.FIRST_FOLDER, "boi.png");
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.addData("doen", "");
                            telemetry.update();
                        }
                    } catch (IOException e) {
                        telemetry.addData("fale","");
                        telemetry.update();
                    }
                }
            }
        }));

        while(opModeIsActive()) {sleep(1);telemetry.update();}

        while (opModeIsActive()) {

            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    if (bitmap != null) {
                        int[] locations = getLocations(bitmap, STONE_LOCATIONS_RED);
                        telemetry.addData("First stone", locations[0]);
                        telemetry.addData("Second stone", locations[1]);
                        telemetry.update();
                    }
                }
            }));
        }

    }

    public int[] locations;

    public void storeLocations(int[] locations) {

        //Pattern A
        if (locations[0] == 3) {
            this.locations = new int[]{0, 3};
        }

        //Pattern B
        else if (locations[0] == 0) {
            this.locations = new int[]{1, 4};
        }

        //Pattern C
        else if (locations[0] == 1) {
            this.locations = new int[]{2, 5};
        }
    }

    public void captureLocations(final int[][] coordsArr){
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    storeLocations(getLocations(bitmap, coordsArr));
                }
            }
        }));
    }



    public int[] getLocations(Bitmap frame, int[][] coordsArr){
        int bestFirstLocation = -1;
        double bestDifference = 0;
        for (int firstLocation = 0; firstLocation < 3; firstLocation++) {
            int firstPixel = frame.getPixel(coordsArr[firstLocation][0], coordsArr[firstLocation][1]);
            int firstR = Color.red(firstPixel);
            int firstG = Color.green(firstPixel);
            int firstB = Color.blue(firstPixel);
            int selAvgR = (int)(firstR);
            int selAvgG = (int)(firstG);
            int selAvgB = (int)(firstB);
            int othAvgR = 0;
            int othAvgG = 0;
            int othAvgB = 0;
            int nOth = 0;
            for (int i = 0; i < 3; i++) {
                if (i != firstLocation) {
                    nOth++;
                    int thisPixel = frame.getPixel(coordsArr[i][0], coordsArr[i][1]);
                    othAvgR += Color.red(thisPixel);
                    othAvgG += Color.green(thisPixel);
                    othAvgB += Color.blue(thisPixel);

                }
            }
            othAvgR /= nOth;
            othAvgG /= nOth;
            othAvgB /= nOth;
            double distance = 0;
            distance += Math.abs(selAvgR - othAvgR);
            distance += Math.abs(selAvgG - othAvgG);
            distance += Math.abs(selAvgB - othAvgB);
            if (distance > bestDifference) {
                bestDifference = distance;
                bestFirstLocation = firstLocation;
            }
        }
        return new int[] {bestFirstLocation};
    }


    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

    }

}
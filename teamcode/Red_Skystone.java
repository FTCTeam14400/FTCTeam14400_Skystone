package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *
 */
@Autonomous(name= "Red Skystone", group="Sky autonomous")
//@Disabled
public class Red_Skystone extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private Servo   stonearm = null;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 3f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 3f/8f+offsetY};
    private static float[] rightPos = {5f/8f+offsetX, 3f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private AndroidTextToSpeech androidTextToSpeech;

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: Neverest 40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .77 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static double           status                  = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        backleft = hardwareMap.get(DcMotor.class, "lb");
        backright = hardwareMap.get(DcMotor.class, "rb");
        frontright = hardwareMap.get(DcMotor.class, "rf");
        frontleft = hardwareMap.get(DcMotor.class, "lf");
        stonearm = hardwareMap.get(Servo.class, "skystone");
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                backleft.getCurrentPosition(),
                backright.getCurrentPosition(),
                frontleft.getCurrentPosition(),
                frontright.getCurrentPosition());
        telemetry.update();

        androidTextToSpeech = new AndroidTextToSpeech();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        androidTextToSpeech.initialize();
        // Set the language and country codes.
        androidTextToSpeech.setLanguageAndCountry("en", "US");

        // Wait for user to push start button.
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            phoneCam.closeCameraDevice();



            if (valLeft == 0){
                androidTextToSpeech.speak("Skystone is left!");
                strafeLeft(.7,41 , 2.5);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.7, 22,2);
                forward(.6,45,2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.7,70,2);
                strafeLeft(.7,25,2);
                backward(.8,5,2);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.7, 22,2);
                forward(.6,70,2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.7,12,2);
                strafeLeft(1,10,2);


                stop();
            }
            else if (valMid == 0){
                androidTextToSpeech.speak("Skystone is middle!");
                strafeLeft(.7,41 , 2.5);
                forward(.6, 7, 2);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.6, 22,2);
                forward(.6, 40, 2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.8, 55, 2);
                backward(.7, 11, 2);
                strafeLeft(.7, 25, 2);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.7, 22, 2);
                forward(.6, 70, 2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.8, 20, 2 );
                strafeLeft(1,20,2);



                stop();
            }
            else if (valRight == 0){
                androidTextToSpeech.speak("Skystone is right!");
                strafeLeft(.7,41 , 2.5);
                forward(.6, 11, 2);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.6, 22,2);
                forward(.6, 32, 2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.8, 51, 2);
                backward(.7, 3, 2);
                strafeLeft(.7, 23, 2);
                stonearm.setPosition(.47);
                sleep(2000);
                strafeRight(.7, 22, 2);
                forward(.6, 60, 2);
                stonearm.setPosition(0);
                sleep(2000);
                backward(.8, 17obs, 2 );
                strafeLeft(1,20,2);



                stop();
            }
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }

    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
    public void forward(double spd, double fwd,double timeoutS) {
        encoderDrive(spd, fwd, fwd, fwd, fwd, timeoutS);
        sleep(200);
    }
    public void backward(double spd, double back,double timeoutS) {
        encoderDrive(spd, -back, -back, -back, -back, timeoutS);
        sleep(200);
    }
    public void turnRight(double spd, double turn, double timeoutS) {
        encoderDrive(spd, turn, -turn, turn, -turn,  timeoutS);
        sleep(200);
    }
    public void turnLeft(double spd, double turn,double timeoutS ) {
        encoderDrive(spd, -turn, turn, -turn, turn,  timeoutS);
        sleep(200);
    }
    public void strafeRight(double spd, double strafe, double timeoutS){
        encoderDrive(spd, -strafe, strafe, strafe, -strafe, timeoutS);

    }
    public void strafeLeft(double spd, double strafe, double timeoutS){
        encoderDrive(spd, strafe, -strafe, -strafe, strafe, timeoutS);

    }
    public void encoderDrive(double speed, double BackleftInches, double BackrightInches, double FrontleftInches, double FrontrightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleft.getCurrentPosition() + (int)(BackleftInches * COUNTS_PER_INCH) ;
            newBackRightTarget = backright.getCurrentPosition() + (int)(BackrightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleft.getCurrentPosition() + (int)(FrontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontright.getCurrentPosition() + (int)(FrontrightInches * COUNTS_PER_INCH);
            backleft.setTargetPosition(newBackLeftTarget);
            backright.setTargetPosition(newBackRightTarget);
            frontleft.setTargetPosition(newFrontLeftTarget);
            frontright.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        backleft.getCurrentPosition(),
                        backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleft.setPower(0);
            backright.setPower(0);
            frontleft.setPower(0);
            frontright.setPower(0);

            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


}

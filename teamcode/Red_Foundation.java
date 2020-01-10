package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;


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
@Autonomous(name= "Red Foundation", group="Sky autonomous")
//@Disabled
public class Red_Foundation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private Servo   stonearm = null;
    private Servo   foundationarm = null;



    private AndroidTextToSpeech androidTextToSpeech;





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
       foundationarm = hardwareMap.get(Servo.class, "foundationarm");
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


        androidTextToSpeech.initialize();
        // Set the language and country codes.
        androidTextToSpeech.setLanguageAndCountry("en", "US");

        // Wait for user to push start button.
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
           backward(.7, 25, 2);
           foundationarm.setPosition(0);
           sleep(2000);
           forward(.7, 44, 2);

           foundationarm.setPosition(0.5);
           sleep(1000);

           strafeRight(1,70,3);



           stop();




            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

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
    public void strafeRight(double spd, double strafe, double timeoutS){
        encoderDrive(spd, -strafe, strafe, strafe, -strafe, timeoutS);

    }
    public void strafeLeft(double spd, double strafe, double timeoutS){
        encoderDrive(spd, strafe, -strafe, -strafe, strafe, timeoutS);

    }
    public void turnRight(double spd, double turn, double timeoutS) {
        encoderDrive(spd, turn, -turn, turn, -turn,  timeoutS);
        sleep(200);
    }
    public void turnLeft(double spd, double turn,double timeoutS ) {
        encoderDrive(spd, -turn, turn, -turn, turn,  timeoutS);
        sleep(200);
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

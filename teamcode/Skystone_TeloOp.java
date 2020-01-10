package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;




@TeleOp(name="Skystone TeleOp", group="League_1")

public class Skystone_TeloOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rf = null;
    private DcMotor lf = null;
    private DcMotor rb = null;
    private DcMotor lb = null;
    private DcMotor m1;
    private DcMotor rotate;
    private Servo stonelock;
    private DcMotor m2;
    private ColorSensor color1;
    private CRServo s1;
    private CRServo s2;
    private DcMotor lift;


  
    @Override
    public void runOpMode() {
    

        rb  = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb" );
        m1 = hardwareMap.dcMotor.get("m1");
        rotate = hardwareMap.dcMotor.get("rotate");
        stonelock = hardwareMap.servo.get("stonelock");
        m2 = hardwareMap.dcMotor.get("m2");
        color1 = hardwareMap.colorSensor.get("color1");
        s1 = hardwareMap.crservo.get("s1");
        s2 = hardwareMap.crservo.get("s2");
        lift = hardwareMap.dcMotor.get("lift");


                      

       
 lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    

rf.setDirection(DcMotor.Direction.REVERSE);
rb.setDirection(DcMotor.Direction.REVERSE);


    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    waitForStart();
    runtime.reset();
    
    while (opModeIsActive()) {
            
    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
    double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
    double rightX = -gamepad1.right_stick_x;
    final double v1 = r * Math.cos(robotAngle)+ rightX;
    final double v2 = r * Math.sin(robotAngle) - rightX;
    final double v3 = r * Math.sin(robotAngle) + rightX;
    final double v4 = r * Math.cos(robotAngle) - rightX;

    lf.setPower(v1);
    rf.setPower(v2);
    lb.setPower(v3);
    rb.setPower(v4);

        rotate.setPower(gamepad2.left_stick_x);

        
        lift.setPower(gamepad2.right_stick_y);
        
        if(gamepad2.right_bumper){
            rotate.setPower(.5);
            
        }
        else if (gamepad2.left_bumper){
            rotate.setPower(-.5);
            
        }
        else {
            rotate.setPower(0);
        }
        
        if (gamepad2.a) {
          stonelock.setPosition(0);
        } 
        else if (gamepad2.b) {
          stonelock.setPosition(0.65);
        }
      
        if (color1.alpha() >= 9000) {
          s1.setPower(0);
          s2.setPower(0);
          m1.setPower(0);
          m2.setPower(0);
        } 
        else {
          s1.setPower(1);
          s2.setPower(-1);
          m1.setPower(-1);
          m2.setPower(-1);
        }
        
         if (gamepad2.x) {
          m1.setPower(1);
          m2.setPower(1);
        } 
        
         /* 
        if (gamepad1.left_bumper) {
          m1.setPower(-1);
          m2.setPower(-1);
        } 
        else {
          m1.setPower(0);
          m2.setPower(0);
        }
        */
        
        telemetry.addData("Color", color1.alpha());
        telemetry.update();

  
        

/*
if(gamepad2.dpad_up){
    frontStone.setPosition(0);
    
}

if(gamepad2.dpad_down){
    backStone.setPosition(1);
    
}

if(gamepad2.dpad_right){
    backStone.setPosition(.18);
    
}

if(gamepad2.dpad_left){
    frontStone.setPosition(.5);
    
}

if(gamepad2.a) {
    horizontallift.setPosition(0);

}
else {
    horizontallift.setPosition(.5);

}
    horizontallift.setPosition(.5);



if(gamepad1.right_bumper){
 in1.setPower(1);
in2.setPower(-1);   
}

else {
    in1.setPower(0);
in2.setPower(0);
}
if(gamepad1.left_bumper){
 in1.setPower(-1);
in2.setPower(1);   
}

else {
    in1.setPower(0);
in2.setPower(0);
}

*/

 



        }
    }
}



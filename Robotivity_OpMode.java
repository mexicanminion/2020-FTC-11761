package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Robotivity_OpMode", group = "Robotivity")

public class Robotivity_OpMode extends OpMode {

    private DcMotor motorFR= null;
    private DcMotor motorFL= null;
    private DcMotor motorBR= null;
    private DcMotor motorBL= null;
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;
    private DcMotor spinboi = null;
    private Servo grabber1 = null;
    private Servo grabber2 = null;
    private Servo blockServo = null;
    private Servo blockServoTwo = null;



    public void init(){

        motorBL = hardwareMap.get(DcMotor.class,"BLMotor");
        motorFL = hardwareMap.get(DcMotor.class,"FLMotor");
        motorBR = hardwareMap.get(DcMotor.class,"BRMotor");
        motorFR = hardwareMap.get(DcMotor.class,"FRMotor");
        rightIntake = hardwareMap.get(DcMotor.class,"Right Intake");
        leftIntake = hardwareMap.get(DcMotor.class,"Left Intake");
        spinboi = hardwareMap.get(DcMotor.class,"spin");
        grabber1 = hardwareMap.get(Servo.class,"Grabber 1");
        grabber2 = hardwareMap.get(Servo.class,"Grabber 2");
        blockServo = hardwareMap.get(Servo.class,"Block");
        blockServoTwo = hardwareMap.get(Servo.class,"Block2");



        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){

        motorFL.setPower(gamepad1.left_stick_y);
        motorBL.setPower(gamepad1.left_stick_y);
        motorFR.setPower(gamepad1.right_stick_y);
        motorBR.setPower(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) {
            motorFL.setPower(1);
            motorBL.setPower(-1);
            motorFR.setPower(-1);
            motorBR.setPower(1);
        }

        if (gamepad1.right_bumper) {
            motorFL.setPower(-1);
            motorBL.setPower(1);
            motorFR.setPower(1);
            motorBR.setPower(-1);
        }

        if (gamepad2.b){
            rightIntake.setPower(1);
            leftIntake.setPower(-1);
        }
        rightIntake.setPower(0);
        leftIntake.setPower(0);

        if (gamepad2.a){
            rightIntake.setPower(-1);
            leftIntake.setPower(1);
        }
        rightIntake.setPower(0);
        leftIntake.setPower(0);

        if(gamepad2.dpad_down){
            grabFound();
        }

        if(gamepad2.dpad_up){
            letGoFound();
        }

        if(gamepad1.y){
            blockServo.setPosition(.5);
            blockServoTwo.setPosition(.3);
        }

        if(gamepad2.x){
            spinboi.setPower(0);
        }

        if(gamepad2.y){
            spinboi.setPower(-.8);
        }
    }


//this makes a cool boy
    private void letGoFound(){
        grabber1.setPosition(.9);
        grabber2.setPosition(-.9);


    }

    private void grabFound(){
        grabber1.setPosition(-.6);
        grabber2.setPosition(.6);
    }


}
//you know me, Di Dao was here! Oh YEAH!
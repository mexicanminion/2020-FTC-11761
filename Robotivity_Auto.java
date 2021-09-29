package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.List;


@Autonomous(name = "Robotivity Main Auto", group = ("Robotivity"))
public class Robotivity_Auto extends LinearOpMode {

    //set up motors
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private Servo grabber1 = null;
    private Servo grabber2 = null;
    private Servo blockServo = null;
    private Servo blockServoTwo = null;

    //set up sensors
    private AnalogInput Color = null;
    private AnalogInput Side = null;

    //data storage BLANK
    private int VisionResult = 0;
    private int feildNumber = 0;


    //gyro IMU
    BNO055IMU imu;
    BNO055IMU imu2;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    private int robotHeading = 0;
    private int robotHeading2 = 0;

    //data for encoders
    private double pi = 3.1415;
    private double ticksPerRev = 1120;//360; //40:1 motors
    private double wheelDiameter = 10;//in cm
    private double DistancePerTick = (pi *wheelDiameter) / ticksPerRev;
    private int driveBaseMotors = 4;
    private double fudge = 1.07;
    private double straf_fudge = 0.695;
    private double straf_fixFL = 1.11;
    private double straf_fixFR = 1.11;
    private double straf_fixBL = 1;
    private double straf_fixBR = 1;

    //vision
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AS5Paxn/////AAABmTmNqZt0CU5ntQbbqJokc6hp3fUR41J7N81dn/IigaWyHyinI3UoFokjKYYwayLnq+NEtZ7F5XRY7FFYe1EC5eST+kjxT+xSMAsmHMNVtcINyIjiO3/RZKWiQjwn1vVw6ygz7pdw+zKMQGQA9b8Yh+gv7uE3lSms5sM9qwMULyGT+ntR44ESkOBJl+XMk7X9Bxfv8maBSsaCztqChOghJ06hT3bmJ1f5D0jQ35CX0S+xoU8bJi/yQO8QdefYO509qv05tx0kNP3i2wPK8QexncHhjRYr0z+t2+XG7/eQLNmOxJCtnpbtEn/Jp99KfSCrFFXLSPxiAYbrktcc0TpPa+8M3BhnOc9a5TkEZQN2DgE9";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //color
    private ColorSensor colorFR;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        motorBL = hardwareMap.get(DcMotor.class, "BLMotor");
        motorFL = hardwareMap.get(DcMotor.class, "FLMotor");
        motorBR = hardwareMap.get(DcMotor.class, "BRMotor");
        motorFR = hardwareMap.get(DcMotor.class, "FRMotor");
        grabber1 = hardwareMap.get(Servo.class, "Grabber 1");
        grabber2 = hardwareMap.get(Servo.class, "Grabber 2");
        blockServo = hardwareMap.get(Servo.class, "Block");
        blockServoTwo = hardwareMap.get(Servo.class,"Block2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu1");
        colorFR = hardwareMap.get(ColorSensor.class, "colorFR");
        Color = hardwareMap.get(AnalogInput.class, "Color");
        Side = hardwareMap.get(AnalogInput.class, "Side");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Motors done");
        telemetry.update();

        resetEncoders();

        blockServo.setPosition(.7);
        blockServoTwo.setPosition(-.2);
        grabber1.setPosition(.9);
        grabber2.setPosition(-.9);
        feildNumber = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
        imu2.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        while (!isStopRequested() && !imu2.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addLine("Gyros Done");
        telemetry.update();

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;
        colorFR.enableLed(bLedOn);


        telemetry.addLine("Color sensor done");
        telemetry.update();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addLine("Vision done");
        telemetry.update();

        if (Color.getVoltage() >= 1.65) {
            telemetry.addLine("Color red");
                if (Side.getVoltage() >= 1.65) {
                telemetry.addLine("Side Foundation");
                feildNumber = 1;
            } else if (Side.getVoltage() < 1.65) {
                telemetry.addLine("Side SkyStone");
                feildNumber = 2;
            }
        } else if (Color.getVoltage() < 1.65) {
            telemetry.addLine("Color Blue");
            if (Side.getVoltage() >= 1.65) {
                telemetry.addLine("Side Foundation");
                feildNumber = 3;
            } else if (Side.getVoltage() < 1.65) {
                telemetry.addLine("Side SkyStone");
                feildNumber = 4;
            }
        }

        telemetry.addLine("everything done");
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();




        waitForStart();




        telemetry.addData("Mode", "running");
        telemetry.update();


        SkyAndFoundBlue();


        /*if(feildNumber == 1){
            telemetry.addLine("doing red foundation");
            telemetry.update();
            redBuildSite();
        }else if(feildNumber == 2){
            telemetry.addLine("doing red skystone");
            telemetry.update();
            redOneQuarry();
        }else if(feildNumber == 3){
            telemetry.addLine("doing blue foundation");
            telemetry.update();
            blueBuildsite();
        }else if(feildNumber == 4){
            telemetry.addLine("doing blue skystone");
            telemetry.update();
            blueOneQuarry();
        }else if(feildNumber == -1){

        }*/

    }

    private void redOneQuarry(){
        driveHeading(-9.5,0,.5);
        scanSkyStone();
        grabSkystoneRed();
        redSkyStoneTwo();
        redOnePark();

    }

    private void blueOneQuarry(){
        driveHeading(-9.5,0,.5);
        scanSkyStone();
        grabSkyStoneBlue();
        //blueSkyStoneTwo();
        blueOnePark();
    }

    private void SkyAndFoundBlue(){
        driveHeading(-9.5,0,.5);
        scanSkyStone();
        grabSkyStoneBlue();
        driveHeading(-4,0,.7);
        driveHeading(6,0,.7);
        FoundWithSkyBlue();
    }

    private void grabSkystoneRed() {

        if(VisionResult == 1){
            telemetry.addLine("vision = 1");
            telemetry.update();
            driveHeading(-3,0,.6);
            strafWithGyro(5,0,.6);
            blockServoTwo.setPosition(.8);
            driveHeading(-1,0,.6);
            sleep(500);
            driveHeading(5,0,.7);
            //strafWithGyro(-2,0,.5);
            //rotate(-80,.5);
            strafWithGyro(-50,0,.6);
            blockServoTwo.setPosition(-.2);
        }
        else if(VisionResult == 2){
            telemetry.addLine("vision = 2");
            telemetry.update();
            left(2,.5);
            driveHeading(-3,0,.5);
            blockServoTwo.setPosition(1);
            sleep(500);
            driveHeading(4,0,.7);
            rotate(-80,.4);
            driveHeading(-25,0,.6);
            blockServoTwo.setPosition(.3);
        }
        else if(VisionResult == 3){
            telemetry.addLine("vision = 3");
            telemetry.update();
            right(3,.5);
            driveHeading(-3,0,.5);
            blockServo.setPosition(-.7);
            sleep(500);
            driveHeading(4,0,.7);
            rotate(-80,.4);
            driveHeading(-25,0,.6);
            blockServo.setPosition(.3);
        }
    }

    private void grabSkyStoneBlue(){

        if(VisionResult == 1){
            telemetry.addLine("vision = 1");
            telemetry.update();
            strafWithGyro(5,0,.5);
            driveHeading(-3,0,.5);
            blockServoTwo.setPosition(1);
            sleep(500);
            driveHeading(4,0,.7);
            strafWithGyro(25,0,.5);
            //rotate(80,.4);
            //driveHeading(-25,0,.5);
            blockServoTwo.setPosition(.3);
        }
        else if(VisionResult == 2){
            telemetry.addLine("vision = 2");
            telemetry.update();
            strafWithGyro(9,0,.5);
            driveHeading(-3,0,.5);
            blockServo.setPosition(.15);
            sleep(500);
            driveHeading(6,0,.7);
            strafWithGyro(34,0,.5);
            //rotate(80,.4);
            //driveHeading(-32,0,.6);
            blockServo.setPosition(.2);
        }
        else if(VisionResult == 3){
            telemetry.addLine("vision = 3");
            telemetry.update();
            strafWithGyro(2,0,.5);
            driveHeading(-3,0,.5);
            blockServo.setPosition(.15);
            sleep(500);
            driveHeading(6,0,.7);
            strafWithGyro(55,0,.5);
            //rotate(80,.4);
            //driveHeading(-35,0,.6);
            blockServo.setPosition(.2);
        }
    }

    private void redSkyStoneTwo(){

        if(VisionResult == 1){
            strafWithGyro(50,0,.6);
            driveHeading(-1,0,.6);
            strafWithGyro(20,0,.6);
            driveHeading(-3,0,.6);
            blockServoTwo.setPosition(1);
            sleep(500);
            driveHeading(4,0,.5);
            strafWithGyro(-70,0,.6);
            blockServoTwo.setPosition(.3);
        }
        else if(VisionResult == 2){
            driveHeading(25,0,.5);
            rotate(80,.4);
            driveHeading(-2,0,.5);
            right(10,.5);
            driveHeading(-1,0,.5);
            blockServo.setPosition(1);
            sleep(500);
            driveHeading(2,0,.7);
            rotate(-80,.4);
            driveHeading(-45,0,.5);
            blockServo.setPosition(.3);
        }
        else if(VisionResult == 3){
            driveHeading(20,0,.5);
            rotate(80,.4);
            right(2,.5);
            driveHeading(-2,0,.5);
            blockServoTwo.setPosition(-1);
            sleep(500);
            driveHeading(2,0,.7);
            rotate(-80,.4);
            driveHeading(-45,0,.5);
            blockServoTwo.setPosition(.3);
        }

    }

    private void blueSkyStoneTwo(){



    }

    private void blueOnePark(){
        if(VisionResult == -1){
            right(3,.5);
            driveHeading(12,0,1);
        } else {
            driveHeading(10,0,.5);
            left(10,.5);
        }
    }

    private void redOnePark() {
        if(VisionResult == -1){
            //no skystone on first try, retry
            left(1,.5);
            scanSkyStone();
            if(VisionResult == -1){
                //no skystone on second try, retry
                right(2,.5);
                scanSkyStone();
                if(VisionResult == -1){
                    //parks when no stones are detected
                    left(20,.5);
                }else{
                    grabSkystoneRed();
                }
            }else{
                grabSkystoneRed();
            }
        } else { //safe gard if it dosnt see the skystone
            //driveHeading(10, 0,.5);
            strafWithGyro(55,0,.5);
        }
    }

    private void blueBuildsite(){
        right(10,.5);
        driveHeading(-14,0,.8);
        grabFound();
        sleep(1000);
        driveHeading(15,0,1);
        rotate(90,.5);
        letGoFound();
        driveHeading(-10,0,1);
        driveHeading(20,0,1);
        right(5,.5);
    }

    private void redBuildSite(){
        left(10,.5);
        driveHeading(-14,0,.8);
        grabFound();
        sleep(1000);
        driveHeading(15,0,1);
        rotate(-90,.5);
        letGoFound();
        driveHeading(-10,0,1);
        driveHeading(20,0,1);
        left (5,.5);
    }

    private void FoundWithSkyBlue(){
        right(25,.5);
        driveHeading(-14,0,.8);
        grabFound();
        sleep(1000);
        driveHeading(15,0,1);
        rotate(90,.5);
        letGoFound();
        driveHeading(-10,0,1);
        right(5,.5);
        driveHeading(20,0,1);
    }

    private void scanSkyStone(){
        vision();
        sleep(2800);
        VisionResult = (vision());
        CameraDevice.getInstance().stop();
    }

  /* private void strafHeading(int distanceInInches, int targetHeading, double speed){

       int error;
       double currentHeading;
       double speedCorrection = 0;
       double leftSpeed,rightSpeed;
       double leftTop,leftBottom;
       double rightTop,rightBottom;
       double gain = .06;
       int distanceTraveled;
       int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

       resetEncoders();

       if (wantedPosition > 0)
       {
           //Going forwards so make sure the speed is +ve
           speed = Math.abs(speed);
       }
       else
       {
           //Going backwards so make sure the speed is -ve
           speed = -Math.abs(speed);
       }

       do {
           //Find where we are currently pointing
           currentHeading = getAngle();

           //Calculate how far off we are
           error = (int)(currentHeading - targetHeading);

           //Using the error calculate some correction factor
           speedCorrection = error * gain;

           //Adjust the left and right power to try and compensate for the error
           leftSpeed = speed + speedCorrection;
            leftTop = leftSpeed;
            leftBottom = -leftSpeed;
           rightSpeed = speed - speedCorrection;
            rightTop = -rightSpeed;
            rightBottom = rightSpeed;

           //Apply the power settings to the motors
           motorFL.setPower(leftTop);
           motorBL.setPower(leftBottom);
           motorFR.setPower(rightTop);
           motorBR.setPower(rightBottom);

           //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
           distanceTraveled = (motorFL.getCurrentPosition() + motorBR.getCurrentPosition() ) / 2;


       }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

       //Done so turn off the motors
       motorFL.setPower(0);
       motorBL.setPower(0);
       motorFR.setPower(0);
       motorBR.setPower(0);

       //Update the direction we think we are pointing
       robotHeading = targetHeading;
   }*/

    private void right (int distanceInInches,double speed){

        int currentPosition = 0;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

        resetEncoders();

        while ((currentPosition < wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.update();

            motorFL.setPower(speed * straf_fixFL);
            motorFR.setPower(-speed * straf_fixFR);
            motorBL.setPower(-speed * straf_fixBL);
            motorBR.setPower(speed * straf_fixBR);
            currentPosition = (motorFL.getCurrentPosition() + motorBR.getCurrentPosition()) / 2;
        }

        hardStop();

    }

    private void left (int distanceInInches,double speed)
    {

        int currentPosition = 0;
        int wantedPosition = (int) ((-distanceInInches / DistancePerTick) * fudge);

        resetEncoders();

        while ((currentPosition > wantedPosition) && opModeIsActive()){
            telemetry.addData("value of current position", currentPosition);
            telemetry.update();

            motorFL.setPower(-speed * straf_fixFL);
            motorBL.setPower(speed * straf_fixBL);
            motorFR.setPower(speed * straf_fixFR);
            motorBR.setPower(-speed * straf_fixBR);
            currentPosition = (motorFL.getCurrentPosition() + motorBR.getCurrentPosition()) / 2;
        }

        hardStop();

    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motorBL.setPower(leftPower);
        motorFL.setPower(leftPower);
        motorFR.setPower(rightPower);
        motorBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double getAngle2()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void strafWithGyro (double distanceInInches, int targetHeading, double speed)
    {
        int error;
        int error2;
        double currentHeading;
        double currentHeading2;
        double speedCorrection = 0;
        double speedCorrection2 = 0;
        double frontSpeed;
        double backSpeed;
        double frontSpeed2;
        double backSpeed2;
        double gain = .06;
        int distanceTraveled;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* straf_fudge);

        resetEncoders();

        if (wantedPosition > 0)
        {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        }
        else
        {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }

        do {
            //Find where we are currently pointing
            currentHeading = getAngle();
            currentHeading2 = getAngle2();


            //Calculate how far off we are
            error = (int)(currentHeading - targetHeading);
            error2 = (int)(currentHeading2 - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;
            speedCorrection2 = error2 * gain;

            //Adjust the left and right power to try and compensate for the error
            frontSpeed = speed + speedCorrection;
            backSpeed = speed - speedCorrection;
            frontSpeed2 = speed + speedCorrection2;
            backSpeed2 = speed - speedCorrection2;

            //Apply the power settings to the motors
            motorFL.setPower(frontSpeed2 * .8);//1.1
            motorBL.setPower(-backSpeed2 * 0.75);//0.75 is drift correction, higher num faster motor
            motorFR.setPower(-frontSpeed * 0.75);
            motorBR.setPower(backSpeed * .8);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

        }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }

    /*private void leftgyro (double distanceInInches, int targetHeading, double speed)
    {
        int error;
        double currentHeading;
        double speedCorrection = 0;
        double frontSpeed;
        double backSpeed;
        double gain = .06;
        int distanceTraveled;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* straf_fudge);

        resetEncoders();

        if (wantedPosition > 0)
        {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        }
        else
        {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }

        do {
            //Find where we are currently pointing
            currentHeading = getAngle2();

            //Calculate how far off we are
            error = (int)(currentHeading - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;

            //Adjust the left and right power to try and compensate for the error
            frontSpeed = speed + speedCorrection;
            backSpeed = speed - speedCorrection;

            //Apply the power settings to the motors
            motorFL.setPower(-frontSpeed);
            motorBL.setPower(backSpeed);
            motorFR.setPower(speed);
            motorBR.setPower(-speed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

        }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        //Update the direction we think we are pointing
        robotHeading2 = targetHeading;
    }*/

    private void driveHeading(double distanceInInches, int targetHeading, double speed)
    {
        int error;
        double currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;
        double gain = .06;
        int distanceTraveled;
        int wantedPosition = (int) ((distanceInInches / DistancePerTick)* fudge);

        resetEncoders();

        if (wantedPosition > 0)
        {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        }
        else
        {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }

        do {
            //Find where we are currently pointing
            currentHeading = getAngle();

            //Calculate how far off we are
            error = (int)(currentHeading - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speed + speedCorrection;
            rightSpeed = speed - speedCorrection;

            //Apply the power settings to the motors
            motorFL.setPower(leftSpeed);
            motorBL.setPower(leftSpeed);
            motorFR.setPower(rightSpeed);
            motorBR.setPower(rightSpeed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (motorFL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition() + motorBL.getCurrentPosition()) / driveBaseMotors;


//            telemetry.addData(">", "H = %d E=%d C=%f3.3",currentHeading, error,speedCorrection);
//            telemetry.update();

        }while ((Math.abs(distanceTraveled) < Math.abs(wantedPosition)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }

    private int getColor(){

        if(colorFR.blue() >= 68){
            return 1;
        }else if(colorFR.red() >= 90){
            return 2;
        }
        else{
            return -1;
        }

    }


    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("after assigning tfodMonitorViewID");
        telemetry.update();
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        telemetry.addLine("after createTFOB");
        telemetry.update();
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        telemetry.addLine("after loadmodel ");
        telemetry.update();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        telemetry.addLine("before params creation");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addLine("after webcame mapping");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addLine("after classfactory instance creation");
        telemetry.update();

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private int vision() {

        float block1Left;
        float block2left;
        boolean skyStone1 = false;
        boolean skyStone2 = false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()>=2) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                Recognition recognition = updatedRecognitions.get(0);
                block1Left = recognition.getLeft();
                telemetry.addData("object 0 left pos:", block1Left);

                if (recognition.getLabel() == "Skystone") {
                    skyStone1 = true;
                }
                telemetry.addData("object 0 SS? ", skyStone1);

                recognition = updatedRecognitions.get(1);
                block2left = recognition.getLeft();
                telemetry.addData("object 1 left pos:", block2left);


                if (recognition.getLabel() == "Skystone")
                    skyStone2 = true;

                telemetry.addData("object 1 SS? ", skyStone2);
                telemetry.update();

                if (skyStone1) {
                    if (block1Left < block2left){
                        telemetry.addLine("CONFIRM SKYSTONE MIDDLE");
                        telemetry.update();
                        return 2;
                    }
                    else {
                        telemetry.addLine("CONFIRM SKYSTONE RIGHT");
                        telemetry.update();
                        return 3;
                    }
                } else if (skyStone2) {
                    if (block2left < block1Left) {
                        telemetry.addLine("CONFIRM SKYSTONE MIDDLE");
                        telemetry.update();
                        return 2;
                    }
                    else {
                        telemetry.addLine("CONFIRM SKYSTONE RIGHT");
                        telemetry.update();
                        return 3;
                    }
                }
                telemetry.addLine("CONFIRM SKYSTONE LEFT");
                telemetry.update();
                return 1;

            } else return -1;
        }
        return -1;
    }


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastAngles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void letGoFound(){
        grabber1.setPosition(.9);
        grabber2.setPosition(-.9);


    }

    private void grabFound(){
        grabber1.setPosition(-.6);
        grabber2.setPosition(.6);
    }

    private void hardStop(){

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    private void resetEncoders(){
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;
//hi
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class autoLeft extends LinearOpMode
{
    final int highJunction = 10500;
    final int slideClearance = 3000;
    final float servoPole = 195.0F;
    final float servoPick = 30.0F;
    double slideSpeed = 2250.0;//2778 PP/S is max encoder PP/S of Gobilda 117 rpm motor
    double driveSpeed = 2796.0;//2796 PP/S is max encoder PP/S of GoBilda 312 rpm motor
    int armTarget = 0;//as encoder values
    double motor_reduction = 0.2;//for drivetrain
    //hardware classes + names
    Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    DcMotorEx Motor_1;//front left
    DcMotorEx Motor_2;//front right
    DcMotorEx Motor_3;//back left
    DcMotorEx Motor_4;//back right
    DcMotorEx armMotor;
    Gyroscope imu;
    Servo intakeServo;
    CRServo wheelServo;
    DistanceSensor frontDistance;
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;
    int tagID = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeCam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        //hardware intializing
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");//to move the claw, grab the cone.
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        //setting hardware directions, etc
        Motor_1.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_3.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_2.setDirection(DcMotorEx.Direction.FORWARD);
        Motor_4.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        Motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);//reset encoder of slideMotor when slide is fully retracted to encoder = 0
        sleep(50);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("armEncoder", armMotor.getCurrentPosition());
        telemetry.addData("intake servo", intakeServo.getPosition());
        armMotor.setTargetPosition(0);//make sure the slide starts at position = 0
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(slideSpeed);
        //intakeServo.setPosition(servoPole/270.0);//"zero" the intake servo
        //telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            motorsRun(-195, -195, -195, -195);
            //sleep(3000);
            while(tagID == 0){
                detectTag();
                telemetry.addData("tag", tagID);
                telemetry.update();
            }
            //sleep(2000);
            putConeLeft();

            /*if(tagID == 1){

            }
            else if(tagID == 2){

            }
            else if(tagID == 3){

            }*/

        }
    }

    void detectTag(){
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if(detections != null)
        {
            /*telemetry.addData("FPS", camera.getFps());
            telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
            */
            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;


                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections)
                {
                    tagID = detection.id;
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                }
            }

            telemetry.update();
        }

        sleep(20);
    }

    void putConeLeft(){
        motorsRun(-1275, 1275, 1275, -1275);

        motorsRun(-1257, -1257, -1257, -1257);
        sleep(50);
        motorsRun(-540, 540, 540, -540);
        /*while(armMotor.getCurrentPosition() <= highJunction){
        }
        motorsRun(-540, 540, 540, -540);
        intakeServo.setPosition(servoPick/270.0);
        sleep(1500);
        /*while (dist > 15.0 || dist < 13.0 && dist<6000){
            if(dist < 13.0){
                Motor_1.setVelocity(0.25 * motor_reduction);
                Motor_2.setVelocity(0.25 * motor_reduction);
                Motor_3.setVelocity(0.25 * motor_reduction);
                Motor_4.setVelocity(0.25 * motor_reduction);
            }
            else if (dist > 15.0){
                Motor_1.setVelocity(-0.25 * motor_reduction);
                Motor_2.setVelocity(-0.25 * motor_reduction);
                Motor_3.setVelocity(-0.25 * motor_reduction);
                Motor_4.setVelocity(-0.25 * motor_reduction);
            }
            else{
                Motor_1.setVelocity(0.0);
                Motor_2.setVelocity(0.0);
                Motor_3.setVelocity(0.0);
                Motor_4.setVelocity(0.0);
            }
            Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor_4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        wheelServo.setPower(1.0);
        sleep(1000);
        wheelServo.setPower(0.0);*/
        parkRight();
    }

    void parkRight(){
        /*intakeServo.setPosition(servoPole/270.0);
        sleep(1000);
        armTarget = 0;
        slide(armTarget);*/
        if(tagID == 1) {
            motorsRun(3940, -3940, -3940, 3940);
        }
        else if(tagID == 2){
            motorsRun(1800, -1800, -1800, 1800);
        }
        else if(tagID == 3){
            motorsRun(585, -585, -585, 585);
        }
        /*while (armMotor.getCurrentPosition() != 0){

        }*/
    }

    void motorsRun(int a, int b, int c, int d){
        Motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_1.setTargetPosition(a);
        Motor_2.setTargetPosition(b);
        Motor_3.setTargetPosition(c);
        Motor_4.setTargetPosition(d);
        Motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_1.setVelocity(driveSpeed * motor_reduction);
        Motor_2.setVelocity(driveSpeed * motor_reduction);
        Motor_3.setVelocity(driveSpeed * motor_reduction);
        Motor_4.setVelocity(driveSpeed * motor_reduction);
        sleep(calcSleep(Math.abs(a)));
    }

    long calcSleep(int PPRdistance){
        telemetry.addData("sleep", Math.round((PPRdistance/(driveSpeed*motor_reduction)*1000)+1000));
        telemetry.update();
        return (Math.round((PPRdistance/(driveSpeed*motor_reduction)*1000)+1000));
    }

    void slide(int target){//make slide move up/down using encoder values to calculate position
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(armTarget<armMotor.getCurrentPosition()){
            armMotor.setVelocity(slideSpeed*0.75);
        }
        else if (armTarget>armMotor.getCurrentPosition()){
            armMotor.setVelocity(slideSpeed);
        }
    }
}






/*
Kai Song
FTC team 19477 & 20848
2022-2023 PowerPlay
MIT LICENSE

Please do not offend my code. I do not claim to be a great coder, but I do claim to be good at robotics.

Robotics:
Mechanical
Electrical
Computational
Happiness
*/

package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.ArrayList;
//edited #2
@Autonomous
public class justPark extends LinearOpMode
{
    /*final int highJunction = 10500;
    final int slideClearance = 3000;
    final float servoPole = 195.0F;
    final float servoPick = 30.0F;
    double slideSpeed = 2250.0;//2778 PP/S is max encoder PP/S of Gobilda 117 rpm motor

    int armTarget = 0;//as encoder values*/
    double driveSpeed = 2796.0;//2796 PP/S is max encoder PP/S of GoBilda 312 rpm motor
    double motor_reduction = 0.2;//for drivetrain
    //hardware classes + names
    Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    DcMotorEx Motor_1, Motor_2, Motor_3, Motor_4, armMotor;//declare motor
    //Servo intakeServo;
    //CRServo wheelServo;
    //DistanceSensor frontDistance;
    OpenCvWebcam camera;//for april tag
    OpenCvWebcam webcam;//for pole detection
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
    double x, y, z, yaw, pitch, roll;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

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
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;*/
        //hardware intializing
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
       /* intakeServo = hardwareMap.get(Servo.class, "intakeServo");//to move the claw, grab the cone.
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");*/
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
        //telemetry.addData("intake servo", intakeServo.getPosition());
        armMotor.setTargetPosition(0);//make sure the slide starts at position = 0
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
        /*while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }*/
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Mode", "calibrating...");

        //armMotor.setVelocity(slideSpeed);
        //intakeServo.setPosition(servoPole/270.0);//"zero" the intake servo
        //telemetry.setMsTransmissionInterval(50);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {//change to 'if' statement
            /*telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);*/
            //sleep(260);
            //detectTag();
            //telemetry.update();
            motorsRun(-195, -195, -195, -195);
            //sleep(3000);
            while(tagID == 0){
                detectTag();
                telemetry.addData("tag", tagID);
                telemetry.update();
            }
            motorsRun(-1000, -1000, -1000, -1000);//drive forward to the next tile
            //sleep(2000);
            //putConeRight();

            if(tagID == 1){
                motorsRun(1300, -1300, -1300, 1300);
            }
            else if(tagID == 3){
                motorsRun(-1150, 1150, 1150, -1150);
            }
            else if(tagID == 2){
                motorsRun(0, 0, 0, 0);
            }

        }
    }

    void detectTag(){//the following is to read/detect the AprilTags. Please Google what they are if you're not familiar. I'm too lazy to explain.
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        fx = aprilTagDetectionPipeline.fx;

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
                    x = detection.pose.x;
                    y = detection.pose.y;
                    z = detection.pose.z;
                    yaw = Math.toDegrees(detection.pose.yaw);
                    pitch = Math.toDegrees(detection.pose.pitch);
                    roll = Math.toDegrees(detection.pose.roll);
                    /*telemetry.addLine(String.format("\nDetected tag ID=%d", tagID));
                    telemetry.addLine(String.format("Translation X: %.2f meters", x));
                    telemetry.addLine(String.format("Translation Y: %.2f meters", y));
                    telemetry.addLine(String.format("Translation Z: %.2f meters", z-0.3));
                    //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", yaw));
                    //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", pitch));
                    //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", roll));
                    telemetry.addLine(String.format("fx: %.2f", fx));
                    telemetry.addLine(String.format("fy: %.2f", fy));
                    telemetry.addLine(String.format("cx: %.2f", cx));
                    telemetry.addLine(String.format("cy: %.2f", cy));*/
                }
            }

            telemetry.update();
        }

        sleep(20);
    }

    void motorsRun(int a, int b, int c, int d){//operate the 4 drive motors with 4 inputs, one for each motor
        //input sequence: motor 1, motor 2...etc
        //TL - 1; TR - 2; BL - 3; BR - 4;
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

    long calcSleep(int PPRdistance){//calculate how long to make the computer sleep to give the motors time to reach their position
        //IK there's a better way to do this but I haven't bothered trying yet.
        telemetry.addData("sleep", Math.round((PPRdistance/(driveSpeed*motor_reduction)*1000)+1000));
        telemetry.update();
        return (Math.round((PPRdistance/(driveSpeed*motor_reduction)*1000)+1000));
    }

}





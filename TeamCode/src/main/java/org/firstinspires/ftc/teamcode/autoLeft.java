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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class autoLeft extends LinearOpMode {
    //junction height values represented as motor encoder values for GoBilda 117 rpm motor
    //'final' means the variable cannot be changed later
    final int pickup = 3;
    final int groundJunction = 1025;
    final int lowJunction = 1325;//4570
    final int midJunction = 2135;//7700
    final int highJunction = 2850;//10500
    final int slideClearance = 720;
    final float servoPole = 0.0F;
    final float servoPick = 213.0F;
    final float clawOpen = 69.0F;
    final float clawClose = 105.0F;
    double slideSpeed = 2787.0;//2787.625 PP/S is max encoder PP/S of Gobilda 435 rpm motor
    double driveSpeed = 2796.0;//2796 PP/S is max encoder PP/S of GoBilda 312 rpm motor
    int armTarget = 0;//as encoder values
    //float intakeServoTarget;
    boolean claw = false;//TRUE = closed; FALSE = open (starting position)
    boolean intake = false;//TRUE = pole position; FALSE = pickup (starting position)
    double minContourArea = 2500.0;//minimum area that a contour is counted as a "cone" and not useless
    //center coordinates have TOP LEFT corner as (0,0) - used for openCv contour center
    int centerColumn = 0;//"x" axis
    int centerRow = 0;//"y" axis

    private ElapsedTime clawTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//initialize clawTime Variable with millisecond unit
    private ElapsedTime intakeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//initialize intakeTime Variable with millisecond unit
    private ElapsedTime cycleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//initialize intakeTime Variable with millisecond unit

    //ALL TIMES ARE IN MILLISECONDS

    //motor variables for mecanum drive
    double motor_reduction = 0.25;//for drivetrain
    double motor_1_pwr, motor_2_pwr, motor_3_pwr, motor_4_pwr = 0.0;//declare motor power variables
    double motor_denom;
    int armPos;

    //PID variables
    double previousTime;
    double lastError = 0;

    //hardware classes + names
    Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    DcMotorEx Motor_1, Motor_2, Motor_3, Motor_4, armMotor;//declare motors
    ServoImplEx intakeServo, clawServo;//declare servos
    //Gyroscope imu;
    //USE SERVOIMPLEX (GM0)
    DistanceSensor frontDistance;
    OpenCvWebcam tagCam;
    OpenCvWebcam intakeCam;

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
    @Override
    public void runOpMode() {
        //hardware initialization
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");//to move the claw, grab the cone.
        clawServo = hardwareMap.get(ServoImplEx.class, "clawServo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        //setting hardware directions, etc
        Motor_1.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_3.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_2.setDirection(DcMotorEx.Direction.FORWARD);
        Motor_4.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //set Servo maxRange [500, 2500] microseconds. Enables goBilda servos to get full 300º range
        intakeServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //should reset encoder values to 0
        Motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);//reset encoder of slideMotor when slide is fully retracted to encoder = 0
        sleep(50);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //telemetry.addData("armEncoder", armMotor.getCurrentPosition());
        //telemetry.addData("intake servo", intakeServo.getPosition());
        clawServo.setPosition(clawClose/300.0);//"open" the claw servo
        intakeServo.setPosition(servoPick/300.0);

        //initialize webcams
        //init aprilTag Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
        tagCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "tagCam"), viewportContainerIds[0]);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        tagCam.setPipeline(aprilTagDetectionPipeline);
        tagCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                tagCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//start getting frames from the camera
            }

            @Override
            public void onError(int errorCode) {

                //This will be called if the camera could not be opened
            }
        });
        //init intakeCam
        intakeCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeCam"), viewportContainerIds[1]);
        intakeCam.setPipeline(new polePipeline());
        intakeCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                intakeCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//start getting frames from the camera
            }

            @Override
            public void onError(int errorCode) {
                //This will be called if the camera could not be opened
            }
        });
        slide(30);//move up slightly cuz there's a weird bug somewhere
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //main loop: call functions that do different tasks
        if (opModeIsActive()) {
            //armPos = armMotor.getCurrentPosition();

            motorsRun(195, -195, -195, 195);
            //sleep(3000);
            while(tagID == 0){
                detectTag();
            }
            telemetry.addData("tag", tagID);
            telemetry.update();
            motorsRun(3000, -3000, -3000, 3000);//drive forward to the next tile
            slide(highJunction);
            armTarget = highJunction;
            motorsRun(-250, -250, -250, -250);
            intakeServo.setPosition(servoPole/300.0);
            sleep(3500);
            clawServo.setPosition(clawOpen);
            sleep(2500);
            intakeServo.setPosition(servoPick/300.0);
            motorsRun(300, 300, 300, 300);
            sleep(1000);
            slide(pickup);
            armTarget = pickup;
            motorsRun(-1800, 1800, 1800, -1800);
            //sleep(2000);
            //putConeRight();

            /*if(tagID == 1){
                motorsRun(1300, -1300, -1300, 1300);
            }
            else if(tagID == 3){
                motorsRun(-1150, 1150, 1150, -1150);
            }
            else if(tagID == 2){
                motorsRun(0, 0, 0, 0);
            }*/
        }
    }



    //all custom functions


    void slide(int target){//make slide move up/down using encoder values to calculate position
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(slideSpeed);
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
                    //yaw = Math.toDegrees(detection.pose.yaw);
                    //pitch = Math.toDegrees(detection.pose.pitch);
                    //roll = Math.toDegrees(detection.pose.roll);
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
        return (Math.round((PPRdistance/(driveSpeed*motor_reduction)*1000)+250));
    }

    //below is some stuff i'm experimenting with
    void centerRobotonCone(){
        //center width is 320
        if(centerColumn != 0) {//if it's 0, means there is no contour
            if (centerColumn < 300) {//turn left
                Motor_1.setVelocity(-0.25 * motor_reduction);
                Motor_2.setVelocity(0.25 * motor_reduction);
                Motor_3.setVelocity(-0.25 * motor_reduction);
                Motor_4.setVelocity(0.25 * motor_reduction);
            }
            else if (centerColumn > 340) {//turn right
                Motor_1.setVelocity(0.25 * motor_reduction);
                Motor_2.setVelocity(-0.25 * motor_reduction);
                Motor_3.setVelocity(0.25 * motor_reduction);
                Motor_4.setVelocity(-0.25 * motor_reduction);
            }
            else {//drive to the cone and make sure it is the right distance away, so the intake will grab it
                if(frontDistance.getDistance(DistanceUnit.CM) > 21.5){
                    Motor_1.setVelocity(0.25 * motor_reduction);
                    Motor_2.setVelocity(0.25 * motor_reduction);
                    Motor_3.setVelocity(0.25 * motor_reduction);
                    Motor_4.setVelocity(0.25 * motor_reduction);
                }
                else if (frontDistance.getDistance(DistanceUnit.CM) < 14.5){
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
            }
        }
        Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*double computePID(double input, double setPoint, int kp, int ki, int kd){
        double currentTime = runTime.milliseconds();//get current time in MILLISECONDS
        double error;
        double cumError = 0;
        double elapsedtime;
        double rateError;
        elapsedtime = (currentTime - previousTime);//compute time elapsed from previous computation

        error = setPoint - input;// determine error
        cumError += error * elapsedtime;                // compute integral
        rateError = (error - lastError)/elapsedtime;   // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;                //PID output
        previousTime = currentTime;
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
    }*/

    /*double computePD(double input, double setPoint, int kp, int kd){
        double currentTime = runTime.milliseconds();//get current time in MILLISECONDS
        double error;
        double cumError = 0;
        double elapsedtime;
        double rateError;
        elapsedtime = (currentTime - previousTime);//compute time elapsed from previous computation

        error = setPoint - input;// determine error
        rateError = (error - lastError)/elapsedtime;   // compute derivative

        double out = kp*error + kd*rateError;                //PID output
        previousTime = currentTime;
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
    }*/

    double remapRange(double val, double old_min, double old_max, double new_min, double new_max) {
        //remaps a value from one range to another range.
        /*Args: val: A number form the old range to be rescaled.
                old_min: The inclusive 'lower' bound of the old range.
                old_max: The inclusive 'upper' bound of the old range.
                new_min: The inclusive 'lower' bound of the new range.
                new_max: The inclusive 'upper' bound of the new range.*/
        double old_range = (old_max - old_min);
        double new_value;
        double new_range;
        if (old_range == 0.0) {
            new_value = new_min;
        }
        else {
            new_range = (new_max - new_min);
            new_value = (((val - old_min) * new_range) / old_range) + new_min;
        }
        return new_value;
    }


    //pole detection processing pipeline
    class polePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        int width = 640;//width of each camera frame
        int height = 480;//height of each camera frame

        Point textAnchor;
        Scalar green = new Scalar(0, 255, 0);//define the RGB value for green

        @Override
        public void init(Mat mat)
        {
            textAnchor = new Point(20, 400);//define where to put the text onto each camera frame
        }

        @Override
        public Mat processFrame(Mat input) {
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen (we will draw stuff onto the input mat and return it)

            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);//convert frame from RGB 2 HSV
            //**IMPORTANT** - HSV color ranges (0-180, 0-255, 0-255)!!!
            //for the HUE value, divide the 0-260 range by 2!!!
            Scalar lowHSV = new Scalar(20, 80, 80); // lower bound HSV for yellow
            Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
            Mat thresh = new Mat();

            // We'll get a black and white image. The white regions represent the cones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            //WE MADE A BLACK/WHITE MASK
            Core.inRange(mat, lowHSV, highHSV, thresh);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);//find the contours from the MASK
            //telemetry.addData("contours", contours);
            //telemetry.update();
            int largestContourArea = 0;
            int largestContour = 0;//largestContour is the INDEX. use contours.get(largestContour) to get the contour
            //find the largest contour, which we assume is the cone we're looking for
            if (contours.size() > 0) {
                for (int i = 0; i < contours.size(); i++) {
                    if (Imgproc.contourArea(contours.get(i)) > largestContourArea && Imgproc.contourArea(contours.get(i)) > minContourArea) {
                        largestContourArea = (int) Imgproc.contourArea(contours.get(i));
                        largestContour = i;
                    }
                }
                if (Imgproc.contourArea(contours.get(largestContour)) > minContourArea) {//check if the largest contour is bigger than the minimum area, so we don't find dumb contours
                    //use the largest contour's moments to find the contour center
                    Moments moments = new Moments();
                    moments = (Imgproc.moments(contours.get(largestContour)));
                    centerRow = (int) Math.round((double) (moments.get_m01() / moments.get_m00()));
                    centerColumn = (int) Math.round((double) (moments.get_m10() / moments.get_m00()));
                    //if a good contour is found, draw the contour onto the initial image, add a text line saying which camera we're looking at, and the contour's area
                    Imgproc.drawContours(input, contours, largestContour, new Scalar(0, 0, 255), 5);//draw the contour in the opposite color (blue cone --> red, red cone --> blue)
                    Imgproc.putText(input, String.format("Camera: intake cam. Area: %d", largestContourArea), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is and the contour area
                    //Mat finish = new Mat();
                    //telemetry.addData("center row: ", centerRow);
                    //telemetry.addData("center Column: ", centerColumn);
                    //telemetry.update();
                    Imgproc.circle(input, new Point(centerColumn, centerRow), 10, new Scalar(255, 255, 0), -1);//draw yellow dot at contour center
                } else {
                    //if no good contour is found, just add a text line saying which camera we're looking at
                    Imgproc.putText(input, String.format("Camera: intake cam."), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is if there is no contour
                }
            }
            else {
                //if no good contour is found, just add a text line saying which camera we're looking at
                Imgproc.putText(input, String.format("Camera: intake cam."), textAnchor, Imgproc.FONT_HERSHEY_PLAIN, 2.0, green, 2);//print which camera it is if there is no contour
            }

            //Imgproc.cvtColor(mat, finish, Imgproc.COLOR_HSV2RGB);
            //MUST RELEASE ALL THE MAT'S WE CREATED, TO NOT LEAK MEMORY
            thresh.release();
            hierarchy.release();
            mat.release();
            //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
            //return the initial image, with the text and contours added
            return input;
        }
    }
}
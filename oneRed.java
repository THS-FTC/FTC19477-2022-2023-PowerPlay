/*
Kai Song
FTC team 19477
2022-2023 PowerPlay
MIT LICENSE

Please do not offend my code. I do not claim to be a great coder, but I do claim to be good at robotics.

Robotics:
Mechanical
Electrical
Computational
Happiness
*/
//import FTC packages and all needed libraries for opencv, vuforia, etc
//hi
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.objdetect.QRCodeDetector;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.Date;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp//set code mode to TeleOp (driver control)
public class oneRed extends LinearOpMode {
    //junction height values represented as motor encoder values for GoBilda 117 rpm motor
    //'final' means the variable cannot be changed later
    final int pickup = 3;
    final int groundJunction = 1025;
    final int lowJunction = 2215;//4570
    final int midJunction = 3870;//7700
    final int highJunction = 5575;//10500
    final int slideClearance = 3000;
    final float servoPole = 0.0F;
    final float servoPick = 300.0F;
    double slideSpeed = 2794.0;//2794 PP/S is max encoder PP/S of Gobilda 223 rpm motor
    double driveSpeed = 2796.0;//2796 PP/S is max encoder PP/S of GoBilda 312 rpm motor
    int armTarget = 0;//as encoder values
    float intakeServoTarget;
    double minContourArea = 2500.0;//minimum area that a contour is counted as a "cone" and not useless
    //center coordinates have TOP LEFT corner as (0,0) - used for openCv contour center
    int centerColumn = 0;//"x" axis
    int centerRow = 0;//"y" axis

    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);//initialize runTime Variable with millisecond unit
    //ALL TIMES ARE IN MILLISECONDS

    //motor variables for mecanum drive
    double motor_reduction = 0.35;//for drivetrain
    double harrisonDirection = 1.0; //if positive, motors are "forward". if negative, motors are "backwards". let harrison figure out the directions I can't :((
    double motor_1_pwr, motor_2_pwr, motor_3_pwr, motor_4_pwr = 0.0;//declare motor power variables
    double motor_denom;

    //PID variables
    double previousTime;
    double lastError = 0;

    //hardware classes + names
    Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    DcMotorEx Motor_1, Motor_2, Motor_3, Motor_4, armMotor;//declare motors
    ServoImplEx intakeServo, clawServo;//declare servos
    Gyroscope imu;
    //USE SERVOIMPLEX (GM0)
    DistanceSensor frontDistance;
    OpenCvWebcam webcam;

    //inputs from controllers
    double left_stick2_x, left_stick2_y, right_stick2_x, right_stick2_y, left_stick1_y, right_stick1_x, left_stick1_x, right_stick1_y,
            left_trig1, right_trig1, left_trig2, right_trig2;//declare triggers and bumpers
    boolean a1, b1, x1, y1, right_bump2, left_bump2, left_bump1, right_bump1, a2, b2, x2, y2, Dleft2, Dright2, Dup2, Ddown2, Dup1, Ddown1, Dleft1, Dright1;//declare a,b,x,y, Dpad, bumpers buttons

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
        Motor_4.setDirection(DcMotorEx.Direction.REVERSE);
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
        intakeServo.setPosition(servoPick/300.0);//"zero" the intake servo

        //initialize webcams
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "intakeCam"), cameraMonitorViewId);
        webcam.setPipeline(new conePipeline());//tell the camera which image processing pipeline to send images to
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//start getting frames from the camera
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        slide(20);//move up slightly cuz there's a weird bug somewhere
        telemetry.addData("Status", "Initialized");
        telemetry.addData("intakeServo PWM Range: ", intakeServo.getPwmRange());
        telemetry.addData("harrison direction: ", harrisonDirection);
        telemetry.update();
        waitForStart();

        //main loop: call functions that do different tasks
        while (opModeIsActive()) {
            normal_motor();////mecanum wheel motor control maths using encoder velocity control
            getCone();//operate the slide to preset positions and rotate the intake
            //cam();//camera statistics
            claw();//operate the claw
            telemetry.addData("arm", armMotor.getCurrentPosition());
            telemetry.update();//send telemetry data to driver hub
            //DO NOT PUT A TELEMETRY UPDATE IN ANY OTHER FUNCTION PLEASE MAY GOD HELP YOU IF YOU DO
        }
    }



    //all custom functions
    void getCone(){
        //read controller inputs
        left_trig2 = this.gamepad2.left_trigger;
        right_trig2 = this.gamepad2.right_trigger;
        Dup2 = this.gamepad2.dpad_up;
        Ddown2 = this.gamepad2.dpad_down;
        x2 = this.gamepad2.x;
        a2 = this.gamepad2.a;
        b2 = this.gamepad2.b;
        y2 = this.gamepad2.y;

        if (left_trig2 > 0.0 || right_trig2 > 0.0){//if either left/right trigger are pressed, execute manual slide height adjustments
            if(left_trig2 > 0.0 && armTarget > 0){
                armTarget -= 30;
                sleep(50);
                slide(armTarget);
            }
            else if(right_trig2 > 0.0 && armTarget < 10000){
                armTarget += 30;
                sleep(50);
                slide(armTarget);
            }
        }
        //spin the servo to the right position as slide is going to preset position
        if ((armMotor.getCurrentPosition() >= slideClearance) & (intakeServo.getPosition() != (intakeServoTarget/300.0))){
            intakeServo.setPosition(intakeServoTarget/300.0);
        }

        //execute preset slide heights if the buttons are pressed
        if (x2){
            slide(pickup);
            intakeServoTarget = servoPick;
        }
        else if(a2){
            slide(lowJunction);
            //intakeServoTarget = servoPole;
        }
        else if(b2){
            slide(midJunction);
            intakeServoTarget = servoPole;
        }
        else if(y2){
            slide(highJunction);
            intakeServoTarget = servoPole;
        }
        //turn the intake servo 180 deg
        if(Dup2){
            intake(0);
        }
        else if (Ddown2){
            intake(1);
        }
    }

    void normal_motor(){//rest please help it's not working!
        //read controller inputs
        right_stick1_x = -this.gamepad1.right_stick_x;
        left_stick1_x = this.gamepad1.left_stick_x;
        left_stick1_y = -this.gamepad1.left_stick_y;
        right_bump1 = this.gamepad1.right_bumper;

        if (right_bump1){//harrisonDirection mapped to drive controller right bumper
            harrisonDirection = harrisonDirection * -1.0;
        }
        //drivetrain (somewhere you can learn more about the math to control the mecanum wheels - GOOGLE IT! IDK)
        motor_denom = Math.max(Math.abs(left_stick1_y) + Math.abs(left_stick1_x) + Math.abs(right_stick1_x), 1.0);
        motor_1_pwr = (left_stick1_y + left_stick1_x + right_stick1_x)/motor_denom;//LF
        motor_2_pwr = (left_stick1_y - left_stick1_x - right_stick1_x)/motor_denom;//RF
        motor_3_pwr = (left_stick1_y - left_stick1_x + right_stick1_x)/motor_denom;//LB
        motor_4_pwr = (left_stick1_y + left_stick1_x - right_stick1_x)/motor_denom;//LR
        Motor_1.setVelocity(motor_1_pwr * driveSpeed * motor_reduction * harrisonDirection);//use setVelocity not setPower to use the encoder and run the robot with velocity (more accurate!!!)
        Motor_2.setVelocity(motor_2_pwr * driveSpeed * motor_reduction * harrisonDirection) ;
        Motor_3.setVelocity(motor_3_pwr * driveSpeed * motor_reduction * harrisonDirection);
        Motor_4.setVelocity(motor_4_pwr * driveSpeed * motor_reduction * harrisonDirection);
        Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//use these statements to run using encoder - NEEDED
        Motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void claw(){//open/close claw
        right_bump2 = this.gamepad2.right_bumper;
        left_bump2 = this.gamepad2.left_bumper;

        if(right_bump2){
            clawServo.setPosition(90.0/300.0);
        }
        else if(left_bump2){
            clawServo.setPosition(50.0/300.0);
        }
    }

    void slide(int target){//make slide move up/down using encoder values to calculate position
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(armTarget<armMotor.getCurrentPosition()){
            armMotor.setVelocity(slideSpeed*0.75);
        }
        else if (armTarget>armMotor.getCurrentPosition()){
            armMotor.setVelocity(slideSpeed*0.9);
        }
    }

    void intake(int pos){//turn the entire intake mechanism around 180 degrees
        if (armMotor.getCurrentPosition() >= slideClearance){//EVERYTHING GOOD
            if(pos == 1){
                intakeServo.setPosition(servoPole/300.0);
            }
            else if (pos == 0){
                intakeServo.setPosition(servoPick/300.0);
            }
            /*else if (pos == 2){
                slide(slideClearance);
                intakeServo.setPosition(servoPole/300.0);
            }*/
        }
        else{
            slide(slideClearance);
        }
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

    void cam(){
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        //if(gamepad1.a)
        //{
            /*
             * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
             * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
             * if the reason you wish to stop the stream early is to switch use of the camera
             * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
             * (commented out below), because according to the Android Camera API documentation:
             *         "Your application should only have one Camera object active at a time for
             *          a particular hardware camera."
             *
             * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
             * but it doesn't hurt to call it anyway, if for no other reason than clarity.
             *
             * NB2: if you are stopping the camera stream to simply save some processing power
             * (or battery power) for a short while when you do not need your vision pipeline,
             * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
             * it the next time you wish to activate your vision pipeline, which can take a bit of
             * time. Of course, this comment is irrelevant in light of the use case described in
             * the above "important note".
             */
            //webcam.stopStreaming();
            //webcam.closeCameraDevice();
        //}
    }

    double computePID(double input, double setPoint, int kp, int ki, int kd){
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
    }

    double computePD(double input, double setPoint, int kp, int kd){
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
    }

    //cone detection processing pipeline
    class conePipeline extends OpenCvPipeline
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
            Scalar lowHSV = new Scalar(150, 5, 5); // lower bound HSV for red
            Scalar highHSV = new Scalar(180, 255, 255); // higher bound HSV for red
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

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}



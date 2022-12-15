/*
- BLUE TEAM CODE; if red, select the proper file
- 1 is first
- float & double + int & long used interchangeably
- 1: forward drive, swideways to the left, radial left, forward intake
- diagonal drive: 1(TL), 2(TR), 3(BL), 4(BR)
- all times is ms

- point execution sequence 
    1: find marker position
    2: put block on correct hub level
    3: park in warehouse
*/








package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.Date;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous

public class Auto_Blue_Warehouse extends LinearOpMode {
    //variables
    //changeable variables
    double level_1_pos = 33.0;
    double level_2_pos = 49.0;
    double level_3_pos = 59.0;
    double SPARK_speed = 0.5;
    int carousel_runtime = 4500;//carousel turn time for 1 duck in ms
    double motors_velocity = 400.0;
    int intake_time = 3000;//intake spit out time
    //////////////////////
    //hardware classes + names
    private Blinker control_Hub;
    private DcMotorEx Motor_1;//front left
    private DcMotorEx Motor_2;//front right
    private DcMotorEx Motor_3;//back left
    private DcMotorEx Motor_4;//back right
    private Gyroscope imu;
    private CRServo CRServo_1;
    private CRServo CRServo_2;
    private Servo Servo_3;
    private Servo Servo_4;
    private CRServo SPARK;
    //private Camera Webcam_1;
    //private ColorSensor Color_1;
    /////////////////////////
    int encoder1;
    int encoder2;
    int encoder3;
    int encoder4;
    double servo_power;
    double left_stick2_x;
    double left_stick2_y;
    double right_stick2_x;
    double left_stick1_y;
    boolean left_bump1;
    boolean right_bump1;
    boolean left_bump2;
    boolean right_bump2;
    double left_trig1;
    double right_trig1;
    double motor_1_pwr;
    double motor_2_pwr;
    double motor_3_pwr;
    double motor_4_pwr;
    int motor_1_pos;
    double motor_denom;
    boolean a1;
    boolean b1;
    boolean x1;
    boolean y1;
    double servo_3_target;
    double servo_4_target;
    double current_3;
    double current_4;
    double top;
    double bottom;
    double left;
    double right;
    int marker_pos = 0;
    double servo_3_current;
    double servo_4_current;
    boolean marker;
    //ML
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_element2.tflite";
    private static final String[] LABELS = {
        "elem",
        "element"
    };
    private static final String VUFORIA_KEY =
            "AamCXzH/////AAABmae2KDCsNEEKk7mUXYfxq2Qhr4MpBXu294jUcaWjspq5RD5ZPVb4FPwcan3cMqX/dDOrascKGtCQt+GyuRA7FbAGDCeuk8+qiM8IWn0K+vSkGVbTofPROiYBqtb2/7s+Ostn6ne5/BYaXgLAdW6BMjpNTn7N6PmnztaFxtt8CcSAyekSWdh5Kfuyo911ZHEmWpu+KosMOtEDAiX6E/6q3A3im51How9k4KlXk5rv4OXigrGVJrbPW2GihDu4T5mgRU8T2J1O6zbHdRaiUKpnvhRHWT7Uom15vBPqA430g0yg5EoswIFLb9RN9ztGvlvki/A3iO6wTFtxCZ3a7/M1/cFWNofvuqJc82qesu8g0WDj";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    @Override
    public void runOpMode() {
        //hardware intializing
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        CRServo_1 = hardwareMap.get(CRServo.class, "CRServo_1");//carousel servo
        Servo_3 = hardwareMap.get(Servo.class, "Servo_3");//arm
        Servo_4 = hardwareMap.get(Servo.class, "Servo_4");//arm
        SPARK = hardwareMap.get(CRServo.class, "SPARK");//spark mini for intake
        //other things
        Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_3.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_2.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo_3.setDirection(Servo.Direction.REVERSE);
        Servo_4.setDirection(Servo.Direction.FORWARD);
        imu = hardwareMap.get(Gyroscope.class, "imu");
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
        //motor init
        //motor_reset();
        //arm(2);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        
        
        
        
        //main loop
        if (opModeIsActive()) {
            arm(1);
            sleep(500);
            motors_set_position(400, 400, 400, 400, motors_velocity);
            sleep(2000);
            if (find_marker() == true){
                marker_pos = 2;
                telemetry.addData("arm", 2);
                telemetry.update();
            }
            //find in middle mark
            else if (marker_pos == 0){
                motors_set_position(450, -450, -450, 450, motors_velocity);//move to the right to get to the 3rd barcode pos
                sleep(2500);
                if (find_marker() == true){
                    marker_pos = 3;
                    telemetry.addData("arm", 3);
                    telemetry.update();
                }
                else if (marker_pos == 0){
                    marker_pos = 1;
                    telemetry.addData("arm", 1);
                    telemetry.update();
                }
            }
            put_cargo();
            warehouse_park();
                        //duck_carousel();
            //reset + telem 
            //telemetry.update();
            
        }
        
    }
   
    
boolean find_marker(){
    boolean marker = false;
    for (int a = 0; a < 5; a++){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      //telemetry.addData("# Object Detected", updatedRecognitions.size());
      // step through the list of recognitions and display boundary info.
        if (updatedRecognitions != null){
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {// fix the if statements to find where the marker is 
                if (recognition.getLabel() == "element"){
                    marker = true;
                    return true;
                }
            i++;
            } 
        }
    }
    if (marker == true){
        return true;
    }
    else if (marker != true); 
        return false;
}   

void put_cargo(){//3 - put cargo in correct position indicated by shipping element
    if (marker_pos == 2){
        motors_set_position(1100, -1100, -1100, 1100, motors_velocity);//move right and stop before where the alliance hub should be
        sleep(2750);//wait 2.75 sec
    }
    else if (marker_pos != 2){
        motors_set_position(800, -800, -800, 800, motors_velocity);//move right and stop before where the alliance hub should be
        sleep(2000);//wait 1.5 sec
    }
    arm(marker_pos);
    sleep(1000);
    motors_set_position(500, 500, 500, 500, motors_velocity);//move forward to the hub
    sleep(2500);//wait 2.5 sec
    intake();
}

void warehouse_park(){//4 - end of auto period - park in storage space
    motors_set_position(-100, -100, -100, -100, motors_velocity);//move back to align with warehouse
    sleep(1500);//wait 1.5 sec
    motors_set_position(-950, 950, -950, 950, motors_velocity);//turn left around perpendicular to warehouse: 880 - 920
    sleep(2400);//wait 2.4 sec
    motors_set_position(-1050, 1050, 1050, -1050, motors_velocity);//move left against the wall
    sleep(2700);//wait 2.7 sec
    motors_set_position(2750, 2750, 2750, 2750, 600.0);//drive forward into warehouse
    sleep(4600);//wait 4.6 sec
}
    





    
//camera and ML stuffs that I don't really understand!!!    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.7f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
    
// motor control!!!
void motors_off(){//all motors off
    Motor_1.setPower(0);
    Motor_2.setPower(0);
    Motor_3.setPower(0);
    Motor_4.setPower(0);
}
void motors_straight(int direction){
    if (direction == 1){//forward
        Motor_1.setPower(1.0);
        Motor_2.setPower(1.0);
        Motor_3.setPower(1.0);
        Motor_4.setPower(1.0);
    }
}
void motors_sideways(int direction){
    if (direction == 1){//sideways to the left
        Motor_1.setPower(-0.5);
        Motor_2.setPower(0.5);
        Motor_3.setPower(0.5);
        Motor_4.setPower(-0.5);
    }
    else;//sideways to the right
        Motor_1.setPower(0.5);
        Motor_2.setPower(-0.5);
        Motor_3.setPower(-0.5);
        Motor_4.setPower(0.5);
}
void motors_radial_turn(int direction){
    if (direction == 1){//radial turn to the left
        Motor_1.setPower(-0.5);
        Motor_2.setPower(0.5);
        Motor_3.setPower(-0.5);
        Motor_4.setPower(0.5);
    }
    else;
        Motor_1.setPower(0.5);
        Motor_2.setPower(-0.5);
        Motor_3.setPower(0.5);
        Motor_4.setPower(-0.5);
}
void motors_diagonal(int direction){
    if (direction == 1){//diagonal to Top Left
        Motor_1.setPower(0.0);
        Motor_2.setPower(0.5);
        Motor_3.setPower(0.5);
        Motor_4.setPower(0.0);
    }
    else if (direction == 2){//diagonal to Top Right
        Motor_1.setPower(0.5);
        Motor_2.setPower(0.0);
        Motor_3.setPower(0.0);
        Motor_4.setPower(0.5);
    }
    else if (direction == 3){//diagonal to Bottom Left
        Motor_1.setPower(-0.5);
        Motor_2.setPower(0.0);
        Motor_3.setPower(0.0);
        Motor_4.setPower(-0.5);
    }
    else;//diagonal to Bottom Right
        Motor_1.setPower(0.0);
        Motor_2.setPower(-0.5);
        Motor_3.setPower(-0.5);
        Motor_4.setPower(0.0);
}
void motors_set_position(int a, int b, int c, int d, double velocity){
    motors_reset();
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
    Motor_1.setVelocity(velocity);
    Motor_2.setVelocity(velocity);
    Motor_3.setVelocity(velocity);
    Motor_4.setVelocity(velocity);
} 
void motors_reset(){
    Motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor_4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
void motors_telemetry(){
    telemetry.addData("velocity", Motor_1.getCurrentPosition());
    telemetry.addData("position", Motor_2.getCurrentPosition());
    telemetry.addData("position", Motor_3.getCurrentPosition());
    telemetry.addData("position", Motor_4.getCurrentPosition());
}
    
//other function control!!!    
    void carousel_turn(){
        CRServo_1.setPower(1.0);//change to positive for auto blue
        sleep(carousel_runtime);
        CRServo_1.setPower(0.0);
    }
    void intake(){//intake
        SPARK.setPower(SPARK_speed);
        sleep(intake_time);
        SPARK.setPower(0.0);
    }
    void arm(int pos){//arm   ----------- delete telemetry stuffs
        if (pos == 1){//lev 1
            Servo_3.setPosition(level_1_pos/180);
            Servo_4.setPosition(level_1_pos/180+0.1);
        }
        else if (pos == 2){//lev 2
            Servo_3.setPosition(level_2_pos/180);
            Servo_4.setPosition(level_2_pos/180+0.1);
        }
        else if (pos == 3){//lev 3
            Servo_3.setPosition(level_3_pos/180);
            Servo_4.setPosition(level_3_pos/180+0.1);
        }
    }
}



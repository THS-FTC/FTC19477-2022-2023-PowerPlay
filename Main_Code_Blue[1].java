/*
- 1 is first
Controls:
    Gamepad 1 (all random functions):
        left BUMPER BUTTON - reset all carousel and intake servos
        right BUMPER BUTTON - turn on the carousel motor
        left TRIGGER - intake forward
        right TRIGGER - intake backward
        x button - arm 1st position
        a button - arm 2nd position
        b button - arm 3rd position
    Gamepad 2 (drive):
        left analog stick - all controls that make logical
        right analog stick - radial turns in logical order and drift
        right BUMPER BUTTON - turn all motors off

Point sequence 
    1st part:
        score freight to shipping hub level 3
    End Game:
        deliver all ducks
        fully park in warehouse
        if extra time:
            ***try to have shared shipping hub touching floor on our side
*/



package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.Date;
//import com.vuforia.Vuforia;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp

public class Main_Code_Blue extends LinearOpMode {
    //changeable variables
    double level_1_pos = 33.0;
    double level_2_pos = 49.0;
    double level_3_pos = 59.0;
    double ground = 0.0;
    double motor_reduction = 0.4;
    private double initial_arm_pos = 10.0;
    double SPARK_speed = 0.5;
    //hardware classes + names
    private Blinker Control_Hub;
    private DcMotor Motor_1;//front left
    private DcMotor Motor_2;//front right
    private DcMotor Motor_3;//back left
    private DcMotor Motor_4;//back right
    private Gyroscope imu;
    private CRServo CRServo_1;
    private CRServo CRServo_2;
    private Servo Servo_3;
    private Servo Servo_4;
    private CRServo SPARK;
    //variables
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
    double servo_3_current;
    double servo_4_current;
    @Override
    public void runOpMode() {
        //hardware intializing
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotor.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotor.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotor.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotor.class, "Motor_4");
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        //main loop
        while (opModeIsActive()) {
            normal_motor();
            carousel_turn();
            intake();
            //reset();
            arm();
            //reset + telem 
            telemetry.update();
        }
        
    }
    
    
    
    
    
    void normal_motor(){//normal motor control maths + telemetry
        //bumpers
        left_bump2 = this.gamepad2.left_bumper;
        right_bump2 = this.gamepad2.right_bumper;
        //chassis control
        if (right_bump2 == true){//all motors off
            Motor_1.setPower(0);
            Motor_2.setPower(0);
            Motor_3.setPower(0);
            Motor_4.setPower(0);
        }
        else;
            //drivetrain
            right_stick2_x = this.gamepad2.right_stick_x * 0.35;
            left_stick2_x = this.gamepad2.left_stick_x * motor_reduction;
            left_stick2_y = -this.gamepad2.left_stick_y * motor_reduction;
            motor_denom = Math.max(Math.abs(left_stick2_y) + Math.abs(left_stick2_x) + Math.abs(right_stick2_x), 1.0);
            motor_1_pwr = (left_stick2_y + left_stick2_x + right_stick2_x)/motor_denom;//LF
            motor_2_pwr = (left_stick2_y - left_stick2_x - right_stick2_x)/motor_denom;//RF
            motor_3_pwr = (left_stick2_y - left_stick2_x + right_stick2_x)/motor_denom;//LB
            motor_4_pwr = (left_stick2_y + left_stick2_x - right_stick2_x)/motor_denom;//LR
            Motor_1.setPower(motor_1_pwr);
            Motor_2.setPower(motor_2_pwr);
            Motor_3.setPower(motor_3_pwr);
            Motor_4.setPower(motor_4_pwr);
            telemetry.addData("motor_1", motor_1_pwr);
            telemetry.addData("motor_2", motor_2_pwr);
            telemetry.addData("motor_3", motor_3_pwr);
            telemetry.addData("motor_4", motor_4_pwr); 
    }
    void carousel_turn(){
        //carousel CRservo
        right_bump1 = this.gamepad1.right_bumper;
        if (right_bump1 == true){
            CRServo_1.setPower(1.0);//positive for blue
            telemetry.addData("carousel: ", "on"); 
        } 
        else;
            CRServo_1.setPower(0.0);
            telemetry.addData("carousel: ", "off"); 
    }
    void intake(){//intake
        left_trig1 = -this.gamepad1.left_trigger;
        right_trig1 = -this.gamepad1.right_trigger;
        if (left_trig1 != 0.0 & right_trig1 == 0.0){//intake forward
            SPARK.setPower(-1 * SPARK_speed);
            telemetry.addData("intake: ", "forward"); 
        }
        if (right_trig1 != 0.0 & left_trig1 == 0.0){//intake backward
            SPARK.setPower(SPARK_speed);
            telemetry.addData("intake: ", "backward"); 
        }
        else if ((left_trig1 == 0.0) && (right_trig1 == 0.0)) {
            SPARK.setPower(0.0);
        }
    }
    void reset(){//reset servos
        left_bump1 = this.gamepad1.left_bumper;
        if (left_bump1 == true){
            CRServo_1.setPower(0.0);
            SPARK.setPower(0.0);
            telemetry.addData("reset", "yes"); 
        }
    }
    void arm(){//arm
        x1 = this.gamepad1.x;
        a1 = this.gamepad1.a;
        b1 = this.gamepad1.b;
        y1 = this.gamepad1.y;
        if (x1 == true){//lev 1
            Servo_3.setPosition(level_1_pos/180);
            Servo_4.setPosition(level_1_pos/180+0.1);
        }
        else if (a1 == true){//lev 2
            Servo_3.setPosition(level_2_pos/180);
            Servo_4.setPosition(level_2_pos/180+0.1);
        }
        else if (b1 == true){//lev 3
            Servo_3.setPosition(level_3_pos/180);
            Servo_4.setPosition(level_3_pos/180+0.1);
        }
        else if (y1 == true){//ground level
            Servo_3.setPosition(ground/180);
            Servo_4.setPosition(ground/180+0.1);
        }    
    }
}



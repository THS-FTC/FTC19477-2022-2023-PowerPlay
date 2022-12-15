/*
Kai Song
FTC team 20848
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

public class twoRed extends LinearOpMode {
    //junction height values represented as motor encoder values for 4-stage Viper Slide Kit
    int groundJunction = 600;
    int lowJunction = 2900;
    int midJunction = 5400;
    int highJunction = 5400;
    double slideSpeed = 2750.0;//2787 PPR is max encoder PPR of Gobilda 435 rpm motor
    int armTarget = 0;//as encoder values
    double driveSpeed = 2796.0;//2796 PP/S is max encoder PP/S of GoBilda 312 rpm motor
    int slidePosition = 0;
    boolean calibrated = false;
    //hardware classes + names
    private Blinker Control_Hub;//NEEDED - DON'T DELETE!!
    private DcMotorEx Motor_1;//front left
    private DcMotorEx Motor_2;//front right
    private DcMotorEx Motor_3;//back left
    private DcMotorEx Motor_4;//back right
    private DcMotorEx armMotor;
    private Gyroscope imu;
    private Servo clawServo;
    //motor variables for mecanum drive
    //double armSpeed = 4000.0;
    double motor_reduction = 0.4;//for drivetrain
    double motor_1_pwr = 0.0;
    double motor_2_pwr = 0.0;
    double motor_3_pwr = 0.0;
    double motor_4_pwr = 0.0;
    double motor_denom;
    //inputs
    double left_stick2_x;//triggers and bumpers
    double left_stick2_y;
    double right_stick2_x;
    double left_stick1_y;
    double right_stick1_x;
    double left_stick1_x;
    boolean left_bump1;
    boolean right_bump1;
    boolean left_bump2;
    boolean right_bump2;
    double left_trig1;
    double right_trig1;
    double left_trig2;
    double right_trig2;
    boolean a1;//a,b,x,y buttons
    boolean b1;
    boolean x1;
    boolean y1;
    boolean a2;
    boolean b2;
    boolean x2;
    boolean Dleft2;
    boolean Dright2;
    boolean Dup1;
    boolean Ddown1;
    boolean Dleft1;
    boolean Dright1;
    boolean y2;

    @Override
    public void runOpMode() {
        //hardware intializing
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Motor_1 = hardwareMap.get(DcMotorEx.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotorEx.class, "Motor_3");
        Motor_4 = hardwareMap.get(DcMotorEx.class, "Motor_4");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");//to move the claw, grab the cone.
        //other things
        Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_3.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_4.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        /*Motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor_4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        telemetry.addData("Status", "Initialized");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);//reset encoder of slideMotor when slide is fully retracted
        telemetry.addData("armEncoder", armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(slideSpeed);
        waitForStart();

        //main loop
        while (opModeIsActive()) {
            normal_motor();
            claw();
            team_2_arm();
            //telemetry.addData("motor1", left_stick2_y);
            telemetry.update();
        }

    }





    void normal_motor(){//normal motor control maths + telemetry
        right_stick1_x = this.gamepad1.right_stick_x;
        left_stick1_x = this.gamepad1.left_stick_x;
        left_stick1_y = -this.gamepad1.left_stick_y;
        //drivetrain
        if(right_bump1){
            motor_reduction = 0.4;
        }
        else if(left_bump1){
            motor_reduction = 0.2;
        }

        //drivetrain
        motor_denom = Math.max(Math.abs(left_stick1_y) + Math.abs(left_stick1_x) + Math.abs(right_stick1_x), 1.0);
        motor_1_pwr = (left_stick1_y + left_stick1_x + right_stick1_x)/motor_denom;//LF
        motor_2_pwr = (left_stick1_y - left_stick1_x - right_stick1_x)/motor_denom;//RF
        motor_3_pwr = (left_stick1_y - left_stick1_x + right_stick1_x)/motor_denom;//LB
        motor_4_pwr = (left_stick1_y + left_stick1_x - right_stick1_x)/motor_denom;//LR
        Motor_1.setVelocity(motor_1_pwr * driveSpeed * motor_reduction * -1.0);
        Motor_2.setVelocity(motor_2_pwr * driveSpeed * motor_reduction * -1.0);
        Motor_3.setVelocity(motor_3_pwr * driveSpeed * motor_reduction * -1.0);
        Motor_4.setVelocity(motor_4_pwr * driveSpeed * motor_reduction * -1.0);
        Motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.update();
    }

    void claw(){
        left_bump2 = this.gamepad2.left_bumper;
        right_bump2 = this.gamepad2.right_bumper;
        if (left_bump2 == true){
            clawServo.setPosition(150.0/270.0);//position for OPEN CLAW

        }
        else if (right_bump2 == true){
            clawServo.setPosition(200.0/270.0);//position for CLOSED CLAW
        }
    }

    void team_2_arm(){//team 2 arm design with claw and 2 servos
        left_trig2 = this.gamepad2.left_trigger;
        right_trig2 = this.gamepad2.right_trigger;
        telemetry.addData("righttrig", right_trig2);
        if(left_trig2 > 0.0 && armTarget > 0){
            armTarget -= 20;
            armMotor.setTargetPosition(armTarget);
        }
        if(right_trig2 > 0.0 && armTarget <= 3000){
            armTarget += 20;
            armMotor.setTargetPosition(armTarget);
            telemetry.addData("targetpos", armMotor.getTargetPosition());
        }
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(slideSpeed);
        telemetry.addData("armPos: ", armTarget);
        //telemetry.update();
    }
}



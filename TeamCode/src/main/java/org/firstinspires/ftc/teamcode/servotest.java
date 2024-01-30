
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "servotest")
@Config
public class servotest extends LinearOpMode {
    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;
    private CRServo mainArm;
    private Encoder armEncoder;
    public static double RAISED_ART=0;
    public static double RAISED_WRIST=0;
    public static double DEPOSIT_ART=0;
    public static double DEPOSIT_WRIST=0;
    public static double DEPOSIT_ART_ALT=0;
    public static double DEPOSIT_WRIST_ALT=0;


    @Override
    public void runOpMode() {
        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");


        waitForStart();


        while  (opModeIsActive()) {
            garra();
            telemetry.addData("rarm",RClaw.getPosition());
            telemetry.addData("larm",LClaw.getPosition());
            telemetry.addData("art",Articulation.getPosition());
            telemetry.update();
        }
    }
    private void garra(){
        if (gamepad2.a){
            Articulation.setPosition(RAISED_ART);
            Wrist.setPosition(RAISED_WRIST);

        }
        if (gamepad2.b){
            Articulation.setPosition(DEPOSIT_ART);
            Wrist.setPosition(DEPOSIT_WRIST);
        }
        if (gamepad2.x){
            Articulation.setPosition(DEPOSIT_ART_ALT);
            Wrist.setPosition(DEPOSIT_WRIST_ALT);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
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
@Disabled
@TeleOp(name = "servotest")

public class servotest extends LinearOpMode {
    private Servo LClaw;
    private Servo RClaw;
    private Servo Articulation;



    @Override
    public void runOpMode() {
        Articulation = hardwareMap.get(Servo.class,"Articulation");
        RClaw = hardwareMap.get(Servo.class, "RClaw");
        LClaw = hardwareMap.get(Servo.class, "LClaw");


        RClaw.setDirection(Servo.Direction.FORWARD);
        RClaw.setDirection(Servo.Direction.FORWARD);
        Articulation.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        double CurrentFront = 0;

        while  (opModeIsActive()) {
            garra();
            //telemetry.addData("fl",frontLeft.getPower());
            //telemetry.addData("fr",frontRight.getPower());
            //telemetry.addData("bl",backLeft.getPower());
            //telemetry.addData("br",backRight.getPower());
            telemetry.addData("larm",RClaw.getPosition());
            telemetry.addData("rarm",LClaw.getPosition());
            telemetry.addData("art",Articulation.getPosition());
            telemetry.update();
        }
    }
    private void garra(){
        if (gamepad2.a){
            if (gamepad2.x){
                RClaw.setPosition(0.5);
                LClaw.setPosition(0.5);
            }else  if (gamepad2.y){
                RClaw.setPosition(0.46);
                LClaw.setPosition(0.54);
            }

        if (gamepad2.dpad_down) {
            Articulation.setPosition(1);
         } else if (gamepad2.dpad_up){
             Articulation.setPosition(0);
        } else if (gamepad2.dpad_left){
            Articulation.setPosition(0.5);
     }}
        else {
            RClaw.setPosition(RClaw.getPosition());
            LClaw.setPosition(LClaw.getPosition());
            Articulation.setPosition(Articulation.getPosition());
        }
    }
}
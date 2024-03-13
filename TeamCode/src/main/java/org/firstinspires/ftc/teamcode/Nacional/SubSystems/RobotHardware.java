package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RobotHardware {
    TrajectorySequence AStarPath;

    public static DcMotor frontRight;
    public static DcMotor backRight;
    public static DcMotor frontLeft;
    public static DcMotor backLeft;
    public static DcMotor mainArm;
    public static DcMotor CoreEsq;
    public static DcMotor CoreDir;
    public static Servo ArticulationL;
    public static Servo ArticulationR;

    public static Servo LClaw;
    public static Servo RClaw;
    public static Servo Wrist;
    public static Servo PlaneServo;
    //public  Servo servoArticulacao;
    //public  Servo servoGarra;
    public static IMU imu;
    public static DistanceSensor distance;
    public static ColorSensor LeftColorSensor;
    public static ColorSensor RightColorSensor;
    public static DigitalChannel leftRedLed;
    public static DigitalChannel leftGreenLed;
    public static DigitalChannel rightRedLed;
    public static DigitalChannel rightGreenLed;
    YawPitchRollAngles orientation;

    static HardwareMap hardwareMap;
    public static SampleMecanumDrive AutonomousDrive;
    public static SampleMecanumDriveCancelable autodrive;
    public static void setHardwareMap(HardwareMap map){
        hardwareMap = map;
    }

    public static void initAll(int BLUESIDE){
        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");
        CoreEsq= hardwareMap.get(DcMotor.class, "Barra1");
        CoreDir = hardwareMap.get(DcMotor.class, "Barra2");


        ArticulationL = hardwareMap.get(Servo.class,"LArticulation");
        ArticulationR = hardwareMap.get(Servo.class,"RArticulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        PlaneServo = hardwareMap.get(Servo.class,"PlaneServo");

        distance = hardwareMap.get(DistanceSensor.class,"DistanceSensorBack");

        LeftColorSensor = hardwareMap.get(ColorSensor.class,"LeftColorSensor");
        RightColorSensor = hardwareMap.get(ColorSensor.class,"RightColorSensor");

        leftGreenLed= hardwareMap.get(DigitalChannel.class,"led1");
        leftRedLed= hardwareMap.get(DigitalChannel.class,"led2");
        rightGreenLed= hardwareMap.get(DigitalChannel.class,"led3");
        rightRedLed = hardwareMap.get(DigitalChannel.class,"led4");

        leftRedLed.setMode(DigitalChannel.Mode.OUTPUT);
        leftGreenLed.setMode(DigitalChannel.Mode.OUTPUT);
        rightRedLed.setMode(DigitalChannel.Mode.OUTPUT);
        rightGreenLed.setMode(DigitalChannel.Mode.OUTPUT);


        CoreEsq.setDirection(DcMotorSimple.Direction.FORWARD);
        CoreDir.setDirection(DcMotorSimple.Direction.REVERSE);

        CoreDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        RClaw.setDirection(Servo.Direction.FORWARD);

        /*
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        /*
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setPower(-0.2);
        AutonomousDrive = new SampleMecanumDrive(hardwareMap);
        AutonomousDrive.setPoseEstimate(PoseStorage.currentPose);
        //DriveBase.initGraph(BLUESIDE);

    }
    public static void startup(){
        //inicializa o robô para o teleop

        CoreDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        HangRobot.hideHooks();

        PlaneServo.setPosition(0);

        mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setPower(0);

        double CurrentFront = 0;
        autodrive = new SampleMecanumDriveCancelable(hardwareMap);
        autodrive.setPoseEstimate(PoseStorage.currentPose);

        ArmMovement.currentArmGoal=0;
        ArmMovement.finalArmGoal=0;
        HangRobot.currentDesiredHeight=0;
        HangRobot.goalhang=0;

    }
    public static void setArtPosition(double ArtPos){
        ArticulationL.setPosition(ArtPos);
        ArticulationR.setPosition(1-ArtPos+ArmMovement.LEFT_ARTICULATION_DIFFERENCE);
    }
    public static void moveArm(double powerArm){
        mainArm.setPower(powerArm);
    }
    public static void setAirplanePosition(double planePos){
        PlaneServo.setPosition(planePos);
    }
    public static void setWristPos(double wristPos){
        Wrist.setPosition(wristPos);
    }
    public static void setDrivebasePower(double fL, double fR, double bL, double bR,double maxSpeedMultiplier){
        frontRight.setPower(fR* maxSpeedMultiplier);
        frontLeft.setPower(fL * maxSpeedMultiplier);
        backRight.setPower(bR * maxSpeedMultiplier);
        backLeft.setPower(bL  * maxSpeedMultiplier);
    }
}

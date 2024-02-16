package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "distsentest")

public class distsentest extends LinearOpMode {
    TrajectorySequence AStarPath;

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor mainArm;
    private DcMotor CoreEsq;
    private DcMotor CoreDir;

    private AnalogInput distanceS;
    private AnalogInput distanceS2;
    private DistanceSensor distSS;
    private DistanceSensor distSS2;
    private DistanceSensor distSS3;
    private DistanceSensor distSS4;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;




    /**
     * This function is executed when the opmode is executed
     */
    @Override
    public void runOpMode() {

        //inicializa hardware
        distanceS = hardwareMap.get(AnalogInput.class,"ddist");
        distanceS2 = hardwareMap.get(AnalogInput.class,"ddist2");
        distSS = hardwareMap.get(DistanceSensor.class,"distSensor");
        distSS2 = hardwareMap.get(DistanceSensor.class,"distSensor2");
        distSS3 = hardwareMap.get(DistanceSensor.class,"distSensor3");
        distSS4 = hardwareMap.get(DistanceSensor.class,"distSensor4");


        //inicializa o robô para o teleop




        waitForStart();

        //grafo é gerado para uso no teleop

        while  (opModeIsActive()) {
            telemetry.addData("sensor0",distSS.getDistance(DistanceUnit.CM));
            telemetry.addData("sensor1",distSS2.getDistance(DistanceUnit.CM));
            telemetry.addData("sensor2",distSS3.getDistance(DistanceUnit.CM));
            telemetry.addData("sensor3",distSS4.getDistance(DistanceUnit.CM));

            telemetry.addData("analog",distanceS.getVoltage());
            telemetry.addData("analog2",distanceS2.getVoltage());
            telemetry.update();

        }
    }


}

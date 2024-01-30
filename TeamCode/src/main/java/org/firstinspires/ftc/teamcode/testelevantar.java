package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "testelevanda")

public class testelevantar extends LinearOpMode {

    private DcMotor CoreDir;
    private DcMotor CoreEsq;

    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    PID pid_core_esq = new PID(0.01,0,0,0);
    PID pid_core_dir = new PID(0.01,0,0,0);
    PID pid_turn = new PID(0,0,0,0);
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    //PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    PID pid_braco = new PID(0.01,0,0,0);
    double prevtime=0;
    double Artpos =0;
    public static List<Node> allNodes= new ArrayList<>();
    double goalX=0;

    /**
     * This function is executed when this Op Mode is selected.
     */

    SampleMecanumDriveCancelable autodrive;
    @Override
    public void runOpMode() {

        //inicializa hardware
        CoreEsq= hardwareMap.get(DcMotor.class, "Barra1");
        CoreDir = hardwareMap.get(DcMotor.class, "Barra2");

        CoreEsq.setDirection(DcMotorSimple.Direction.FORWARD);
        CoreDir.setDirection(DcMotorSimple.Direction.FORWARD);

        CoreDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //define variaveis essenciais e inicializa funções
        waitForStart();


        while  (opModeIsActive()) {
            if (gamepad2.right_bumper){
                CoreDir.setPower(gamepad2.left_trigger);
                CoreEsq.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_bumper){
                CoreDir.setPower(-gamepad2.left_trigger);
                CoreEsq.setPower(-gamepad2.right_trigger);
            } else {
                CoreDir.setPower(0);
                CoreEsq.setPower(0);
            }
        }
    }
}

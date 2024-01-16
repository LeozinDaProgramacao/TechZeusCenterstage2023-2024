package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Extras extends LinearOpMode
{
    public void addTelemetry(Telemetry rtelemetry){
            /*Description:
            this funtion adds all debugging data needed to the telemetry board
            */
        rtelemetry.addData("CurrentExecMode", DV.currentBaseExecMode);
        rtelemetry.addData("pb", DV.powerbraco);
        rtelemetry.addData("pos",DV.frontLeft.getCurrentPosition());
        rtelemetry.addData("distance",DV.distanceSensor.getDistance(DistanceUnit.CM));
        rtelemetry.addData("error",DV.pid_braco.error);
        rtelemetry.addData("gm2 lt",gamepad2.left_stick_x);
        rtelemetry.addData("gm2 rt",gamepad2.right_stick_x);
        rtelemetry.update();
        //imu
            /*
            telemetry.addData("Delta Angle", DeltaAngle);//quanto a base deslocou em radianos desde o ultimo reset
            telemetry.addData("turn PID detected Error", pid_turn.error*180/Math.PI);
            telemetry.addData("current angle",CurrentAngle*180/Math.PI);
            telemetry.addData("currentfront",CurrentFront);
            telemetry.addData("cumError",pid_turn.cummulativeError);
            /* */
        //valores crus (direto do gamepad)
            /*
            telemetry.addData("raw FB",-1*gamepad1.left_stick_y);
            telemetry.addData("raw LR",gamepad1.left_stick_x);
            telemetry.addData("raw TR",raw_turn);
            telemetry.addData("1",fL);
            telemetry.addData("2",fR);
            telemetry.addData("3",bL);
            telemetry.addData("4",bR);
            //telemetry.addData("multiplier",maximo);//garante que a velocidade maxima seja 1/**/
        //velocidades aplicadas nas rodas
            /*
            telemetry.addData("delay=",getRuntime()-prevtime);
            prevtime=getRuntime();
            telemetry.addData("fL",fL);
            telemetry.addData("fR",fR);
            telemetry.addData("bL",bL);
            telemetry.addData("bR",bR);
            telemetry.addData("armpos",backRight.getCurrentPosition());
            telemetry.addData("goalbraco",goalbraco);
            telemetry.addData("powerbraco",powerbraco);
            //telemetry.addData("LARM power",LArm.getPower());
            //telemetry.addData("RARM power",RArm.getPower());
            /**/
        //Encoders
            /*
            telemetry.addData("bL encoder",backLeft.getCurrentPosition());
            telemetry.addData("bR encoder",backRight.getCurrentPosition());
            telemetry.addData("fL encoder",frontLeft.getCurrentPosition());
            telemetry.addData("fR encoder",frontRight.getCurrentPosition());
            /**/
        //telemetry.update(); // adiciona tudo da telemetria
    }
    public void HardwareMapAll(){
        DV.autodrive = new SampleMecanumDrive(hardwareMap);
        //inicializa hardware
        DV.imu = hardwareMap.get(IMU.class, "imu");
        DV.frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        DV.backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        DV.frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        DV.backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        DV.mainArm = hardwareMap.get(CRServo.class, "BRACO_TEMP_MUDAR_DPS");

        DV.Articulation = hardwareMap.get(Servo.class,"Articulation");
        DV.LClaw = hardwareMap.get(Servo.class,"LClaw");
        DV.RClaw = hardwareMap.get(Servo.class,"RClaw");
        DV.Wrist = hardwareMap.get(Servo.class,"Wrist");
        DV.mainArm = hardwareMap.get(CRServo.class, "BRACO_TEMP_MUDAR_DPS");



        //corrige orientação dos motores
        DV.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        DV.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        DV.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        DV.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //falsifica um encoder pro braço
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

        DV.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DV.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DV.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DV.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //define variaveis essenciais e inicializa funções
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}

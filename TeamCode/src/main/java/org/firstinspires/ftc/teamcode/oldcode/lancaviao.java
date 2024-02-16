package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@TeleOp (name="lancaviao")
public class lancaviao extends LinearOpMode {
    FtcDashboard maromba;
    public static double KP=0.01;

    public static double KI=0.0006;
    public static double KD=0;
    private DcMotor frontRight;


    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    PID pid_lanca = new PID(0.5,0.000,0.0,0.3);

    @Override
    public void runOpMode() {
        maromba =FtcDashboard.getInstance();
        frontRight = hardwareMap.get(DcMotor.class, "lancaviao");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        TelemetryPacket pack1 = new TelemetryPacket();
        boolean press =false;
        double speed =0;
        while  (opModeIsActive()) {
            pack1.put("armpos",frontRight.getCurrentPosition());
            maromba.sendTelemetryPacket(pack1);


            if (gamepad1.right_bumper&&!press){
                press =!press;
                speed +=0.05;
            } else if((gamepad1.left_bumper)&&(!press)){
                press =!press;
                speed -= 0.05;
            } else if (!gamepad1.right_bumper&&!gamepad1.left_bumper){
                press =!press;
                if (gamepad1.a) {
                    speed =0;
                    frontRight.setPower(0);
                }}

            if (gamepad1.b){
               frontRight.setPower(speed);
            }

            telemetry.addData("pos",frontRight.getCurrentPosition());
            telemetry.addData("speedd ",speed);
            telemetry.addData("speed",frontRight.getPower());
            telemetry.update();
        }
    }

}

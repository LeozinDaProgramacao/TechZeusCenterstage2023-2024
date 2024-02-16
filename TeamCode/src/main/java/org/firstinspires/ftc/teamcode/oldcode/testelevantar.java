package org.firstinspires.ftc.teamcode.oldcode;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "testelevanda")
@Disabled

public class testelevantar extends LinearOpMode {

    public static double PVARIATION = 0.001;
    public static double LVARIATION =6;

    private DcMotor CoreDir;
    private DcMotor CoreEsq;
    public static double KP1=0;
    public static double KP2=0;
    public static double KI1=0;
    public static double KI2=0;

    PID pid_core_esq = new PID(0.05,0,0,0);
    PID pid_core_dir = new PID(0.05,0,0,0);

    double goalhang=0;

    /**
     * This function is executed when this Op Mode is selected.
     */

    SampleMecanumDriveCancelable autodrive;
    private int finalgoalhang=0;

    @Override
    public void runOpMode() {

        //inicializa hardware
        CoreEsq= hardwareMap.get(DcMotor.class, "Barra1");
        CoreDir = hardwareMap.get(DcMotor.class, "Barra2");

        CoreEsq.setDirection(DcMotorSimple.Direction.FORWARD);
        CoreDir.setDirection(DcMotorSimple.Direction.REVERSE);

        CoreDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //define variaveis essenciais e inicializa funções
        waitForStart();


        while  (opModeIsActive()) {


            if (gamepad2.right_trigger>0.5&&gamepad2.left_trigger>0.5){
                finalgoalhang = 250;
            } if (gamepad2.x&&gamepad2.y){
                finalgoalhang = 800;
            } if (gamepad2.b){
                finalgoalhang=0;
            }
            //goalhang+= PVARIATION*(finalgoalhang-goalhang);
            if (((goalhang > finalgoalhang - LVARIATION) && (goalhang < finalgoalhang + LVARIATION))) {
                goalhang = finalgoalhang;
            } else if (goalhang>finalgoalhang){
                    goalhang-=LVARIATION;
            } else if (goalhang<finalgoalhang){
                goalhang+=LVARIATION;
            }
            double powEsq= pid_core_esq.CalculatePID(CoreEsq.getCurrentPosition(),goalhang,false);
            double powDir =pid_core_dir.CalculatePID(CoreDir.getCurrentPosition(),goalhang,false);
            CoreEsq.setPower(powEsq);
            CoreDir.setPower(powDir);
            telemetry.addData("left corehex",CoreEsq.getCurrentPosition());
            telemetry.addData("right corehex",CoreDir.getCurrentPosition());
            telemetry.update();
        }
    }
}

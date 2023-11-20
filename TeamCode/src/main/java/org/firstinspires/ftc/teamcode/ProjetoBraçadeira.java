package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


    @TeleOp(name="ProjetoBraçadeira", group="Linear Opmode")

    public class ProjetoBraçadeira extends LinearOpMode {

        // Define motores//
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null; // Motor Esquerdo.
        private DcMotor rightFrontDrive = null; // Motor Direito.


        @Override
        public void runOpMode() {



            // Definição do nome de motores no Driver e Contol Hub:

            leftFrontDrive  = hardwareMap.get(DcMotor.class, "FE");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "FD");

            // Define as manetes:

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

            //FORWARD
            //REVERSE

            // Telemetria do Drive Hub:

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                double max;

                //  Posições dos Botões na manete:

                double axial   = -gamepad1.left_stick_y;  // Botão Esquerdo.
                double yaw     =  gamepad1.right_stick_x; // Botão Direito.


                double leftFrontPower  = -axial  + yaw;
                double rightFrontPower = -axial  - yaw;

                // Compara motor esquerdo com o direito:

                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

                // Define a potênica maxíma que pode ser colocadas nos motores:

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                }

                double ForcaE = leftFrontDrive.getPower();
                double ForcaD = rightFrontDrive.getPower();


                // Onde é aplicada a potência:

                leftFrontDrive.setPower(leftFrontPower*1);
                rightFrontDrive.setPower(rightFrontPower*1);

                // Mais Telemetria do Drive Hub:

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                double strength = rightFrontDrive.getPower();
                telemetry.addData("Force", "%4.2f", strength);


            }
        }}

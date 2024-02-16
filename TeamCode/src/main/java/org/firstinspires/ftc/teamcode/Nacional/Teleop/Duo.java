package org.firstinspires.ftc.teamcode.Nacional.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.HangRobot;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Nacional.Utility.ArtStateMachine;
import org.firstinspires.ftc.teamcode.Nacional.Utility.simpleSwitch;

@Config

@TeleOp (name="DUO >:D")
public class Duo extends LinearOpMode {
    simpleSwitch LClawSwitch = new simpleSwitch();
    simpleSwitch RClawSwitch = new simpleSwitch();
    ArtStateMachine stateMachine = new ArtStateMachine();
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.setHardwareMap(hardwareMap);
        RobotHardware.initAll();
        AirplaneLauncher.resetAirplaneServo();

        waitForStart();

        RobotHardware.startup();

        while (opModeIsActive()){
            loopRobot();
        }
        AirplaneLauncher.resetAirplaneServo();
    }

    public void loopRobot(){

        defineArmHeight();

        ArmMovement.ControlClawTeleop(LClawSwitch.click(gamepad2.left_bumper),RClawSwitch.click(gamepad2.right_bumper));

        ArmMovement.armPIDLoop(gamepad2.b);
        //RobotHardware.mainArm.setPower(gamepad2.left_trigger);

        DriveBase.moveWithIMU(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,gamepad1.a,gamepad1.b);

        AirplaneLauncher.launchAirplane(gamepad1.left_trigger,gamepad1.right_trigger);

        HangRobot.HangLoop(gamepad2.x,gamepad2.y);
    }
    public void defineArmHeight() {

        if (gamepad2.dpad_up) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK);
        }
        if (gamepad2.dpad_left) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_FRONT);
        }
        if (gamepad2.dpad_down) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL1UP);
        }
        if (gamepad2.dpad_right) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
        }
        if (HangRobot.currentDesiredHeight==HangRobot.HOOK_HEIGHT_GRAB||HangRobot.currentDesiredHeight==HangRobot.HOOK_HEIGHT_HANG){
            ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
        }
        if (gamepad1.a||gamepad1.b&&ArmMovement.currentArmState== ArmMovement.ARM_STATE.PIXEL1UP){
            stateMachine.increaseOrLower(gamepad1.a, gamepad1.b);
            switch (stateMachine.height){
                case 0:
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL2UP);
                    break;
                case 1:
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL3UP);
                    break;
                case 2:
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                    break;
                case 3:
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                    break;
                case 4:
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_NO_ARM);
                    break;

            }
        }

    }

}

package org.firstinspires.ftc.teamcode.Nacional.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.HangRobot;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.WebcamAprilTags;
import org.firstinspires.ftc.teamcode.Nacional.Utility.ArtStateMachine;
import org.firstinspires.ftc.teamcode.Nacional.Utility.counterSwitch;
import org.firstinspires.ftc.teamcode.Nacional.Utility.simpleSwitch;
@Config

@TeleOp (name="Duo >:D")
public class Duo extends LinearOpMode {
    simpleSwitch LClawSwitch = new simpleSwitch();
    simpleSwitch RClawSwitch = new simpleSwitch();
    counterSwitch stateMachine = new counterSwitch(4);
    ArtStateMachine sm = new ArtStateMachine();

    public enum BASE_MODE {
        NORMAL, GRAPH;
    }

    public static BASE_MODE currentMode = BASE_MODE.NORMAL;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware.setHardwareMap(hardwareMap);
        RobotHardware.initAll();
        AirplaneLauncher.resetAirplaneServo();
        waitForStart();
        RobotHardware.startup();
        WebcamAprilTags.initAprilTag(hardwareMap);
        while (opModeIsActive()) {
            loopRobot();
        }
        AirplaneLauncher.resetAirplaneServo();
    }

    public void loopRobot() {
        defineArmHeight();
        ArmMovement.ControlClawTeleop(LClawSwitch.click(gamepad2.left_bumper), RClawSwitch.click(gamepad2.right_bumper));
        ArmMovement.armPIDLoop(gamepad2.back);
        Pose2d estimate = WebcamAprilTags.LocateWithAprilTag(telemetry,RobotHardware.autodrive);


        if (RobotHardware.autodrive.getPoseEstimate()==estimate) {
            RobotHardware.autodrive.update();
        } else{
            if (Math.abs(gamepad1.right_stick_y)+Math.abs(gamepad1.left_stick_x)+Math.abs(gamepad1.right_stick_x)<0.1) {
                RobotHardware.autodrive.setPoseEstimate(WebcamAprilTags.LocateWithAprilTag(telemetry, RobotHardware.autodrive));
            } else{
                RobotHardware.autodrive.update();
            }
        }

        //RobotHardware.mainArm.setPower(gamepad2.left_trigger);
        if (currentMode == BASE_MODE.NORMAL) {
            if (gamepad1.x) {
                currentMode = BASE_MODE.GRAPH;
                DriveBase.startGraphMode(true);
            }
            DriveBase.moveWithIMU(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a, gamepad1.b);

        } else if (currentMode == BASE_MODE.GRAPH) {
            if (!DriveBase.loopMoveGraph(
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.dpad_left,
                    gamepad1.dpad_right,
                    RobotHardware.autodrive.getPoseEstimate().getHeading(),
                    gamepad1.b)) {
                currentMode = BASE_MODE.NORMAL;
            }
        }
        AirplaneLauncher.launchAirplane(gamepad1.left_trigger, gamepad1.right_trigger);
        HangRobot.HangLoop(gamepad2.x && gamepad2.y, gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7);

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
            ArmMovement.currentArmState = ArmMovement.ARM_STATE.PIXEL1UP;
            ArmMovement.Artheight = -1;
        }
        if (gamepad2.dpad_right) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
        }
        if (HangRobot.currentDesiredHeight == HangRobot.HOOK_HEIGHT_GRAB || HangRobot.currentDesiredHeight == HangRobot.HOOK_HEIGHT_HANG) {
            ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
        }
        if (gamepad2.dpad_down&&gamepad2.a){
            ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
        }
        if (gamepad2.dpad_down&&gamepad2.b){
            ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL2UP);
        }
    }
}


package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions;


@TeleOp(name="Ballsack")
public class Control extends LinearOpMode {
    Extras ex = new Extras();
    DriveBase drive = new DriveBase();
    ArmMovement armm = new ArmMovement();
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();

        while(opModeIsActive()){
            updateCurrentMode();
            runCurrentMode();
        }
    }
    public void setup(){
        ex.HardwareMapAll(hardwareMap);
        telemetry.addData("Status","initialized");
        telemetry.update();
    }
    void runCurrentMode(){
        switch (DV.currentExecMode){
            case FULL_FUNCTION_MODE:
                drive.manageIMU();
                drive.moveBaseIMU();
                armm.braco();
                armm.ClawControl();
                break;
            case BACKDROP_MODE:
                drive.moveBaseBackdropFull();
                break;
            case HANG_MODE:
                break;
            case LIMITED_MODE:
                drive.moveBaseNoIMU();


        }
    }
    void updateCurrentMode(){
        DV.currentExecMode = DV.ExecMode.FULL_FUNCTION_MODE;
    }
}

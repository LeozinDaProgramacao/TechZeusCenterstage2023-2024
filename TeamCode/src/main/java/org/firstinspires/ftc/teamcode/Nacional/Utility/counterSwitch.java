package org.firstinspires.ftc.teamcode.Nacional.Utility;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;

public class counterSwitch{
    public int maxV=4;
    public int currV=0;
    public counterSwitch(int newMaxV){
        this.maxV = newMaxV;
    }
    private boolean previouslyPressedIncrease=false;

    private boolean currentStateIncrease = false;
    private boolean previouslyPressedDecrease=false;
    private boolean currentStateDecrease = false;
    public void click(boolean pressedincrease, boolean presseddecrease) {
        if (!this.previouslyPressedIncrease && pressedincrease&&currV!=maxV) {
            this.currentStateIncrease = !this.currentStateIncrease;
            ArmMovement.Artheight++;
        }


        if (!this.previouslyPressedDecrease && presseddecrease&&currV!=0) {
            this.currentStateDecrease = !this.currentStateDecrease;
            ArmMovement.Artheight = ArmMovement.Artheight-1;
        }
        this.previouslyPressedIncrease = pressedincrease;
        this.previouslyPressedDecrease = presseddecrease;
    }
}
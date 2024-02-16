package org.firstinspires.ftc.teamcode.Nacional.Utility;

public class ArtStateMachine {
    public int height =0;
    static simpleSwitch increaseSwitch = new simpleSwitch();
    static simpleSwitch decreaseSwitch = new simpleSwitch();
    public void setHeightGround(){
        this.height=0;
    }
    public void increaseOrLower(boolean increase,boolean lower){
        if (increase&&increaseSwitch.click(increase)&&height!=4){
            this.height++;
        } else if (lower&& decreaseSwitch.click(lower)&&height!=0){
            this.height--;
        }

    }
}

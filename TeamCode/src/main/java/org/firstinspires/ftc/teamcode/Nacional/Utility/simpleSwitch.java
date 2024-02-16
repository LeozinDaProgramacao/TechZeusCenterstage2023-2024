package org.firstinspires.ftc.teamcode.Nacional.Utility;

public class simpleSwitch{
    private boolean previouslyPressed=false;
    private boolean currentState = false;
    public boolean click(boolean pressed) {
        if (!previouslyPressed && pressed) {
            this.currentState = !currentState;
        }
        this.previouslyPressed = pressed;
        return currentState;
    }
}
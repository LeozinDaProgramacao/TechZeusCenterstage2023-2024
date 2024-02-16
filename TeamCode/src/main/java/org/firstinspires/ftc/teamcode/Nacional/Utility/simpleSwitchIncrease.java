package org.firstinspires.ftc.teamcode.Nacional.Utility;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;

public class simpleSwitchIncrease{
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
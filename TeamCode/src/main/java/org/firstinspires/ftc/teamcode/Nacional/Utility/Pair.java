package org.firstinspires.ftc.teamcode.Nacional.Utility;

public class Pair {
    public double XPos;
    public double YPos;
    public Pair(double nXPos, double nYPos){
        XPos = nXPos;
        YPos = nYPos;
    }
    public double getX(){
        return XPos;
    }
    public double getY(){
        return YPos;
    }
    void setX(double X){
        XPos=X;
    }
    void setY(double Y){
        YPos = Y;
    }
}
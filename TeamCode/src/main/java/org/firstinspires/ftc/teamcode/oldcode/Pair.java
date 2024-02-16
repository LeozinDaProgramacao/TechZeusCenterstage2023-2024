package org.firstinspires.ftc.teamcode.oldcode;
public class Pair {
    double XPos;
    double YPos;
    Pair(double nXPos,double nYPos){
        XPos = nXPos;
        YPos = nYPos;
    }
    double getX(){
        return XPos;
    }
    double getY(){
        return YPos;
    }
    void setX(double X){
        XPos=X;
    }
    void setY(double Y){
        YPos = Y;
    }
}

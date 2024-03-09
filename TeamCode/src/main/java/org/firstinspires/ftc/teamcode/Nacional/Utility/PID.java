package org.firstinspires.ftc.teamcode.Nacional.Utility;


public class PID {
    public double kP;
    double kI;
    double kD;
    double MaxI;

    public PID(double newkP, double newkI, double newkD, double newMaxI){
        kP =newkP;
        kI = newkI;
        kD = newkD;
        MaxI = newMaxI;
    }
    double goal=0;
    double position=0;
    double max_acceleration = 1;
    double max_speed = 1;
    public double cummulativeError=0;
    double error=0;
    double previous_speed=0;
    double lastError =0;

    public double P(double multiplier){
        return multiplier*(error);
    }



    public double I(double multiplier){
        if (!(multiplier * Math.abs(cummulativeError + error) > MaxI)) {
            cummulativeError += error;
        }
        return multiplier*cummulativeError;
    }


    public double D(double multiplier){
        return multiplier*(error-lastError);
    }



    public double CalculatePID(double current_position,double current_goal){
        goal = current_goal;
        position = current_position;
        error = goal-position;

        double speed = P(kP) + I(kI) + D(kD);

        if (speed >previous_speed+max_acceleration){
            previous_speed += max_acceleration;
            speed = previous_speed;
        }else if (speed < (previous_speed - max_acceleration)){
            previous_speed -= max_acceleration;
            speed = previous_speed;
        }
        if (Math.abs(speed)>max_speed){
            speed = max_speed*Math.abs(speed)/ speed;
        }

        lastError = error;

        return speed;
    }
}
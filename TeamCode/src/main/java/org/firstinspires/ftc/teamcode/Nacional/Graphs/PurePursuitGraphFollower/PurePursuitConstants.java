package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;


import com.acmerobotics.dashboard.config.Config;


@Config
public class PurePursuitConstants {
    public static double Lfc = 15;//look forward constant, aka min distance to point
    public static double PPk = 4;//no idea what this does, I wrote the code and forgot lol
    public static double kP = 0.6;//proportional term for the positional PID
    public static double MAXSPEED = 0.6;
    public static double MAXACCEL =1.4;//test this slowly when I can

    public static int spp = 40;//amount of segments between each point provided by graph, lower for increased performance,
                               // increase for accuracy
}

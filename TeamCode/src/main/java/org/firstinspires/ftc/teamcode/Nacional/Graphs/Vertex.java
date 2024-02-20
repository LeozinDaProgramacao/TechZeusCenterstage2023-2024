package org.firstinspires.ftc.teamcode.Nacional.Graphs;


import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
/**
 *
 * @author carlo
 */
public class Vertex implements Comparable<Vertex>{

    public double f;
    public double g;
    public double XPos;
    public String id;
    public double YPos;
    public List<String> neighbors;
    public Vertex parent;

    @Override
    public int compareTo(Vertex n) {
        return Double.compare(this.f, n.f);
    }

    public Vertex(double XPos,double YPos,String newId){
        this.XPos = XPos;
        this.YPos = YPos;
        this.id = newId;
        this.neighbors = new ArrayList<String>();
    }

    public void addEdge(String id,Vertex newNeighbor){
        neighbors.add(id);
    }
    public void removeEdge(String id){
        neighbors.remove(id);
    }
}


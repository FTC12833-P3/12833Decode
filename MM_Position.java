package org.firstinspires.ftc.teamcode;

public class MM_Position {
    public double x = 0;
    public double y = 0;
    public double heading = 0;

    public MM_Position(double x, double y, double heading) {
        setAll(x, y, heading);
    }

    public double getX(){
        return x; //unit is inches
    }

    public double getY(){
        return y; //unit is inches
    }

    public double getHeading(){
        return heading;
    }

    public void setX(double newX){
        x = newX;
    }

    public void setY(double newY){
        y = newY;
    }

    public void setHeading(double newHeading){
        heading = newHeading;
    }

    public void setAll(double newX, double newY, double newHeading){
        x = newX;
        y = newY;
        heading = newHeading;
    }
}

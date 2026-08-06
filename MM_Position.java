package org.firstinspires.ftc.teamcode;

public class MM_Position {
    private double x = 0;
    private double y = 0;
    private double heading = 0;

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

    public void setX(double x){
        if (Math.abs(x) < 63) { // check if x is valid
            this.x = x;
        }
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class MM_PID_CONTROLLER {
    MM_OpMode opMode;
    private final ElapsedTime loopTime = new ElapsedTime();

    public static double P_COEFF = 0.02083;
    //public static double I_COEFF;
    public static double D_COEFF = 0;

    private double prevError = 0;

    private double P;
    private double I;
    private double D;

    public MM_PID_CONTROLLER(double P_CO_EFF, double I_CO_EFF, double D_CO_EFF){
        P_COEFF = P_CO_EFF;
        //I_COEFF = I_CO_EFF;
        D_COEFF = D_CO_EFF;

        this.opMode = opMode;
    }

    public double getPID(double error){
        P = error * P_COEFF;
        I = 0; //no I for now
        D = (error - prevError) / loopTime.milliseconds() * D_COEFF;
        loopTime.reset();

        prevError = error;
        return P + I + D;
    }

    public double getD() {
        return D;
    }

    public double getI() {
        return I;
    }

    public double getP() {
        return P;
    }
}

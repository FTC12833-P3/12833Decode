package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_PID_CONTROLLER {
    MM_OpMode opMode;
    private final ElapsedTime PIDTimer = new ElapsedTime();

    private double P_COEFF;
    private double I_COEFF;
    private double D_COEFF;

    private double prevError = 0;
    private double integralSum = 0;

    private double P;
    private double I;
    private double D;

    public MM_PID_CONTROLLER(double P_CO_EFF, double I_CO_EFF, double D_CO_EFF){
        P_COEFF = P_CO_EFF;
        I_COEFF = I_CO_EFF;
        D_COEFF = D_CO_EFF;

        this.opMode = opMode;
    }

    public double getPID(double error){
        double dt = PIDTimer.milliseconds(); //delta time
        PIDTimer.reset();

        if (Math.abs(error) < 10) {
            integralSum += (error * dt);
            integralSum = Range.clip(integralSum, -100, 100);
        }

        P = error * P_COEFF;
        I = integralSum * I_COEFF;
        D = ((error - prevError) / dt) * D_COEFF;

        prevError = error;
        return P + I + D;
    }

    public void setPID(double P, double I, double D) {
        setP_COEFF(P);
        setI_COEFF(I);
        setD_COEFF(D);
    }

    public double getP_COEFF(){
        return P_COEFF;
    }

    public double getI_COEFF() {
        return I_COEFF;
    }

    public double getD_COEFF(){
        return D_COEFF;
    }

    public void setP_COEFF(double newPCoEff){
        P_COEFF = newPCoEff;
    }

    public void setI_COEFF(double newICoEff) {
        I_COEFF = newICoEff;
    }

    public void setD_COEFF(double newDCoEff){
        D_COEFF = newDCoEff;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public void resetController() {
        prevError = 0;
        integralSum = 0;
        PIDTimer.reset();
    }

    public double getIntegralSum() {
        return integralSum;
    }
}

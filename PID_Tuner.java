package org.firstinspires.ftc.teamcode;

public class PID_Tuner extends MM_OpMode{

    @Override
    public void runProcedures(){
        while (opModeIsActive()){
            robot.drivetrain.autoRunDrivetrain();
        }
    }
}

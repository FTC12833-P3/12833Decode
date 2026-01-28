package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="PID tune")
public class PID_Tuner extends MM_OpMode{

    @Override
    public void runProcedures(){
        while (opModeIsActive()){
            robot.drivetrain.enableBrakes();
            robot.collector.runCollector();
            robot.drivetrain.autoRunDrivetrain();
            multipleTelemetry.update();
        }
    }
}

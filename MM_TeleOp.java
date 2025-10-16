package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends MM_OpMode{

    @Override
    public void runProcedures(){
        robot.init();

        waitForStart();

        while(opModeIsActive()){
            robot.drivetrain.driveWithSticks();
            robot.collector.runCollector();
            robot.launcher.runLauncher();
            MM_Launcher.runLauncher = true;
            multipleTelemetry.update();
        }
    }
}

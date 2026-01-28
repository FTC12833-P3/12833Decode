package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends MM_OpMode{

    @Override
    public void runProcedures(){
        robot.drivetrain.enableBrakes();
        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            robot.drivetrain.driveWithSticks();
            robot.collector.runCollector();
            robot.launcher.runLauncher();
            MM_Launcher.scoreArtifacts = true;
            robot.drivetrain.navigation.updatePosition();
            multipleTelemetry.addData("alliance", alliance == -1? "blue": "red");

            multipleTelemetry.update();

        }
    }
}

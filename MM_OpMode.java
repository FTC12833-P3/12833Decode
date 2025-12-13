package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public static String scoringLocation = "";
    public static int alliance = -1; //default = blue
    public MM_Spline currentSpline = null;

    public void runOpMode(){
        multipleTelemetry.addData("Status", "Initializing... please wait");
        multipleTelemetry.update();
        initialize();

        while (opModeInInit()){
            multipleTelemetry.addData("Status", "Initialized");
            multipleTelemetry.update();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            robot.drivetrain.navigation.updatePosition();
            robot.launcher.setServerForInit();

            multipleTelemetry.addData("alliance", alliance == -1? "blue": "red");
        }
        runProcedures();
        if(isStopRequested()){
            //TODO add any static variables reset code here
        }
    }

    public abstract void runProcedures();

    public void telemetry(){
        multipleTelemetry.update();
    }

    public void initialize(){
        robot = new MM_Robot(this);
        robot.init();
    }
}

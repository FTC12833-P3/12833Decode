package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    public static int BLUE = -1;
    public static int RED = 1;

    public static String GOAL_SIDE = "Goal Side";
    public static String AUDIENCE_SIDE = "Audience Side";
    public static int startPos;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public static String scoringLocation = "";
    public static int alliance = -1; //default = blue
    public static int motif = -1;
    public MM_Spline currentSpline = null;


    boolean goalSide = true;
    boolean allSpikes = true;
    boolean spike1 = true;
    boolean spike2 = true;
    boolean spike3 = true;
    boolean eliminationMatch = true;

    public enum SETTINGS {
        ALL_SPIKES,
        SPIKE_1,
        SPIKE_2,
        SPIKE_3,
        ELIMINATION_MATCH,
        GOAL_SIDE
    }

    boolean[] settings = {allSpikes, spike1, spike2, spike3, eliminationMatch, goalSide};
    String[] settingsNames = {"AllSpikesEnabled", "spike1Enabled", "spike2Enabled", "spike3Enabled", "Elimination Match", "starting by goal"};
    int currentSetting = 0;

    public void runOpMode(){
        startPos = 1;

        multipleTelemetry.addData("Status", "Initializing... please wait");
        multipleTelemetry.update();

        initialize();

        multipleTelemetry.addLine("Bumpers to change setting");
        multipleTelemetry.addLine("Triggers to toggle true/false");
        multipleTelemetry.addData(settingsNames[currentSetting], settings[currentSetting]);

        while (opModeInInit()){
            multipleTelemetry.addData("Status", "Initialized");
            if(getClass() == MM_Autos.class){
                multipleTelemetry.addLine("Bumpers to change setting");
                multipleTelemetry.addLine("Triggers to toggle true/false");

                multipleTelemetry.addData(settingsNames[currentSetting], settings[currentSetting]);

                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    nextSetting();
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                    previousSetting();
                }
                if((currentGamepad1.right_trigger > 0 &&! (previousGamepad1.right_trigger > 0)) || (currentGamepad1.left_trigger > 0 &&! (previousGamepad1.left_trigger > 0))){
                    settings[currentSetting] = !settings[currentSetting];
                }
            }
            multipleTelemetry.update();
            robot.drivetrain.navigation.initSpikeMarks();

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
            MM_Launcher.scoreArtifacts = false;
            MM_Position_Data.targetPos.setAll(0, 0, 0);

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

    private void nextSetting(){
        if (currentSetting == settings.length - 1){
            currentSetting = 0;
        } else {
            currentSetting += 1;
        }
    }

    private void previousSetting(){
        if (currentSetting < 1){
            currentSetting = 0;
        } else {
            currentSetting -= 1;
        }
    }
}

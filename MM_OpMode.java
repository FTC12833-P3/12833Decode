package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    public static int BLUE = -1;
    public static int RED = 1;

    public static String GOAL_SIDE = "Goal Side";
    public static String AUDIENCE_SIDE = "Audience Side";

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public static String scoringLocation = "";
    public static int alliance = -1; //default = blue
    public static int motif = -1;
    public MM_Spline currentSpline = null;

    boolean chosenStartingLocation = false;

    int goalSide = 1;
    int allSpikes = 1;
    int spike1 = 0;
    int spike2 = 1;
    int spike3 = 2;
    int eliminationMatch = 1;
    int hpCollect = 0;
    ElapsedTime driveTime;

    public enum SETTINGS {
        ALL_SPIKES,
        SPIKE_1,
        SPIKE_2,
        SPIKE_3,
        ELIMINATION_MATCH,
        GOAL_SIDE,
        HP_COLLECT,
    }

    int[] settings = {allSpikes, spike1, spike2, spike3, eliminationMatch, goalSide, hpCollect};
    String[] settingsNames = {"AllSpikesEnabled", "spike1Enabled", "spike2Enabled", "spike3Enabled", "Elimination Match", "starting by goal", "hpCollect"};
    int currentSetting = 0;
    List<MM_Spline> goalSideCollectSplines;
    List<MM_Spline> audienceSideCollectSplines;
    List<MM_Spline> chosenSplineList;

    public void runOpMode(){
        multipleTelemetry.addData("Status", "Initializing... please wait");
        multipleTelemetry.update();

        initialize();

        if(getClass() == MM_Autos.class){
            goalSideCollectSplines = Arrays.asList(null, robot.drivetrain.navigation.goalSideSplineToCollectSecondSpikeMark, robot.drivetrain.navigation.goalSideSplineToCollectThirdSpikeMark);
            audienceSideCollectSplines = Arrays.asList(null, robot.drivetrain.navigation.audienceSideSplineToCollectSecondSpikeMark, robot.drivetrain.navigation.audienceSideSplineToCollectThirdSpikeMark);
            chosenSplineList = settings[SETTINGS.GOAL_SIDE.ordinal()] == 1 ? goalSideCollectSplines: audienceSideCollectSplines;
            driveTime = new ElapsedTime();
        }

//        multipleTelemetry.addLine("Bumpers to change setting");
//        multipleTelemetry.addLine("Triggers to toggle true/false");
//        multipleTelemetry.addData(settingsNames[currentSetting], settings[currentSetting]);

        while (opModeInInit()){
            multipleTelemetry.addData("Status", "Initialized");
            if(getClass() == MM_Autos.class){
                //multipleTelemetry.addLine("Bumpers to change setting");
                //multipleTelemetry.addLine("Triggers to toggle true/false (1 / 0)");
                multipleTelemetry.addData(settingsNames[currentSetting], settings[currentSetting]);

                if (!chosenStartingLocation) {
//                    if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                        nextSetting();
//                    } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
//                        previousSetting();
//                    }
//                    if ((currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0))) {
//                        settings[currentSetting]++;
//                    } else if ((currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0))) {
//                        settings[currentSetting]--;
//                    }
                    if (gamepad1.rightBumperWasPressed()) { //toggle goal or audience side
                        if (settings[SETTINGS.GOAL_SIDE.ordinal()] == 0) {
                            settings[SETTINGS.GOAL_SIDE.ordinal()] = 1;
                        } else {
                            settings[SETTINGS.GOAL_SIDE.ordinal()] = 0;
                        }
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        chosenStartingLocation = true;
                        chosenSplineList = settings[SETTINGS.GOAL_SIDE.ordinal()] == 1? goalSideCollectSplines: audienceSideCollectSplines;
                    }
                }

                multipleTelemetry.addData("selected starting side", settings[SETTINGS.GOAL_SIDE.ordinal()] == 1? "goal": "audience");
                multipleTelemetry.addData("starting side locked in?", chosenStartingLocation? "yes": "no");

                if (chosenStartingLocation) {
                    robot.drivetrain.navigation.initSpikeMarks();
                }

            } else {
                chosenStartingLocation = true;
            }

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

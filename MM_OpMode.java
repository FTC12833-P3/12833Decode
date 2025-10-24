package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;
    public static int BLUE = 1;
    public static int RED = -1;

    public static String GOAL_SIDE = "Goal Side";
    public static String AUDIENCE_SIDE = "Audience Side";
    public static int startPos;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public static String scoringLocation = "";
    public static int alliance = -1;
    public MM_Spline currentSpline = null;

    boolean allSpikes = true;
    boolean spike1 = true;
    boolean spike2 = true;
    boolean spike3 = true;
    boolean[] settings = {allSpikes, spike1, spike2, spike3};
    String[] settingsNames = {"AllSpikesEnabled", "spike1Enabled", "spike2Enabled", "spike3Enabled"};
    int currentSetting = 0;

    public void runOpMode(){
        startPos = 1;

        multipleTelemetry.addData("Status", "Initializing... please wait");
        multipleTelemetry.update();
        initialize();

        while (opModeInInit()){
            multipleTelemetry.addData("Status", "Initialized");
            if(getClass() == MM_Autos.class){
                multipleTelemetry.addData("Alliance", alliance == 1? "Red": "Blue");
                multipleTelemetry.addData("Start Position", startPos == 1? "Goal Side": "Audience Side");
                multipleTelemetry.addData(settingsNames[currentSetting], settings[currentSetting]);

                if (currentGamepad1.x && !previousGamepad1.x){
                    startPos *= -1;
                }

                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    selectNextSpikeMark();
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                    selectPreviousSpikeMark();
                }
            }
            multipleTelemetry.update();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            robot.drivetrain.navigation.updatePosition(true);
            multipleTelemetry.addData("in autos class", "");

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

    private void selectNextSpikeMark(){
        if (currentSetting == settings.length - 1){
            currentSetting = 0;
        } else {
            currentSetting += 1;
        }
    }

    private void selectPreviousSpikeMark(){
        if (currentSetting < 1){
            currentSetting = 0;
        } else {
            currentSetting -= 1;
        }
    }
}

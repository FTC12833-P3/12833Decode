package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class MM_Launcher {
    private MM_OpMode opMode;
    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo pusher;
    private CRServo server;
    //private ColorSensor peephole;
    private ColorSensor launchSensor;
    private ColorSensor lowerLaunchSensor;
    private DistanceSensor rightDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor lowerLeftDistance;
    private MM_PID_CONTROLLER serverPID;


    private AnalogInput serverEncoder;

    private MM_Position projectileTarget = new MM_Position(-65 * MM_OpMode.alliance, -65 * MM_OpMode.alliance, 0); //blue goal pos

    public static double LAUNCHER_CO_EFF = 2.25;
    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double targetLauncherVelocity = 1;
    public static double LOWER_FEED_ARM_POSITION_1 = .6;
    public static double LOWER_FEED_ARM_POSITION_2 = .76;
    public static double LOWER_FEED_ARM_POSITION_3 = .85;
    public static double AXON_ENCODER_CO_EFF = 1;

    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height
    private final double LOWER_FEED_BAR_TOP_POSITION = .8;
    public static double HYPOT_SPEED_THRESHOLD = 23;
    public static double HYPOT_SPEED_CO_EFF = 1.5;


    public static double SERVER_P_CO_EFF = .03;

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    public boolean artifactAtTop = true;
    private boolean serverIsReady = false;
    private boolean launching = false;
    public static int serverStopPoint = 210;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;

        launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
        launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
        launchMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        pusher = opMode.hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(0);
        server = opMode.hardwareMap.get(CRServo.class, "server");
        serverEncoder = opMode.hardwareMap.get(AnalogInput.class, "serverEncoder");
        //setServerForInit(); TODO fix method to make it stop server at right place in init
        //peephole = opMode.hardwareMap.get(ColorSensor.class, "upperFeedSensor");
        launchSensor = opMode.hardwareMap.get(ColorSensor.class, "launchSensor");
        lowerLaunchSensor = opMode.hardwareMap.get(ColorSensor.class, "lowerLaunchSensor");
        rightDistance = opMode.hardwareMap.get(DistanceSensor.class, "rightLaunchSensor");
        leftDistance = opMode.hardwareMap.get(DistanceSensor.class, "launchSensor");
        lowerLeftDistance = opMode.hardwareMap.get(DistanceSensor.class, "lowerLaunchSensor");
        serverPID = new MM_PID_CONTROLLER(SERVER_P_CO_EFF, 0, 0);

        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLauncher() {
        setTargetLauncherVelocity();
        haveArtifactAtTop();


        opMode.multipleTelemetry.addData("launcherTargetSpeed", targetLauncherVelocity);
        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());
        opMode.multipleTelemetry.addData("servoEncoder", getAxonDegrees(serverEncoder));
        opMode.multipleTelemetry.addData("lowerLaunchSensor info", "red: %d green: %d blue: %d alpha: %d",lowerLaunchSensor.red(), lowerLaunchSensor.green(), lowerLaunchSensor.blue(), lowerLaunchSensor.alpha());
        opMode.multipleTelemetry.addData("dist Left", leftDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Right", rightDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Lower Left", lowerLeftDistance.getDistance(DistanceUnit.MM));

        //if (launching){
        if(currentGamepad2.b && !previousGamepad2.b && !artifactAtTop) {

            if (pusher.getPosition() >= LOWER_FEED_ARM_POSITION_3) {
                pusher.setPosition(0);
            } else if (pusher.getPosition() >= LOWER_FEED_ARM_POSITION_2) {
                pusher.setPosition(LOWER_FEED_ARM_POSITION_3);
            } else if (pusher.getPosition() >= LOWER_FEED_ARM_POSITION_1) {
                pusher.setPosition(LOWER_FEED_ARM_POSITION_2);
            } else {
                pusher.setPosition(LOWER_FEED_ARM_POSITION_1);
            }
        }
            opMode.multipleTelemetry.addData("servo pos", pusher.getPosition());
       // }

        if (haveArtifactAtTop() || launching) { //TODO only set velocity once
            launchMotorLeft.setVelocity(targetLauncherVelocity);
            launchMotorRight.setVelocity(targetLauncherVelocity);
        } else {
            launchMotorLeft.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
            launchMotorRight.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
        }

        if (pusher.getPosition() >= LOWER_FEED_ARM_POSITION_1 && !launching && artifactAtTop && currentGamepad2.right_trigger > 0  && Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50) {
//            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION); TODO fix the lower feed arm
            server.setPower(1);
            launching = true;
        }
        //opMode.multipleTelemetry.addData("colors", "red %d, green %d, blue %d", peephole.red(), peephole.green(), peephole.blue());

        if(launching) {
            double serverError = getAxonDegrees(serverEncoder) - serverStopPoint;
            if (getAxonDegrees(serverEncoder) < serverStopPoint) {
                serverIsReady = false;
                server.setPower(serverPID.getPID(serverError > 0? serverError: 0));
            } else if (!serverIsReady) {
                serverIsReady = true;
                launching = false;
                server.setPower(0);
            }
        }
    }

    private void setTargetLauncherVelocity() {
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())) * 0.0254;
        opMode.multipleTelemetry.addData("launchDistance", launchDistance);
        //double metersPerSecond = Math.sqrt(Math.abs((launchDistance * Math.tan(LAUNCHER_ANGLE) - (9.81 * Math.pow(launchDistance, 2)) )/ (2 * FINAL_PROJECTILE_HEIGHT * Math.pow(Math.cos(LAUNCHER_ANGLE), 2))));
        double metersPerSecond = Math.sqrt((9.81 * launchDistance) / Math.sin(LAUNCHER_ANGLE * 2));
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
        targetLauncherVelocity = Math.abs(metersPerSecond * TICKS_PER_METER) * LAUNCHER_CO_EFF;
        if(Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                 projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())) <= HYPOT_SPEED_THRESHOLD){
            targetLauncherVelocity *= HYPOT_SPEED_CO_EFF;
        }
        opMode.multipleTelemetry.addData("hypot", Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY()));
    }

    private boolean haveArtifactAtTop(){
        artifactAtTop = launchSensor.red() <= 10;
        opMode.multipleTelemetry.addData("red", launchSensor.red());
        opMode.multipleTelemetry.addData("green", launchSensor.green());
        opMode.multipleTelemetry.addData("blue", launchSensor.blue());
        opMode.multipleTelemetry.addData("alpha", launchSensor.alpha());
        return true;
    }

    public boolean lowerFeedArmReady(){
        return pusher.getPosition() == 0;
    }

    private double getAxonDegrees(AnalogInput encoder){
        return ((encoder.getVoltage() / 3.3) * 360);
    }

    public void setServerForInit(){
        double error = getAxonDegrees(serverEncoder) - serverStopPoint;

        if(!serverIsReady) {
            if (getAxonDegrees(serverEncoder) < serverStopPoint) {
                server.setPower(serverPID.getPID(error > 0? error: 0));
            } else {
                serverIsReady = true;
                server.setPower(0);
            }
        }
    }
}


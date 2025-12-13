package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.alliance;
import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private DistanceSensor topRightDistance;
    private DistanceSensor topLeftDistance;
    private DistanceSensor bottomLeftDistance;
    private DistanceSensor bottomRightDistance;
    //public static MM_PID_CONTROLLER serverPID;

    private AnalogInput serverEncoder;

    private MM_Position projectileTarget = new MM_Position(-65, 65 * alliance, 0); //goal pos

    public static double LAUNCH_ZONE_CO_EFF_AUDIENCE = 2.5;
    public static double LAUNCH_ZONE_CO_EFF_FIELD_CENTER = 2.25;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_MID = 2.25;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_NEAR = 3.375;

    public static double LAUNCH_ZONE_BOUNDARY_AUDIENCE = 100;
    public static double LAUNCH_ZONE_BOUNDARY_FIELD_CENTER = 80;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_MID = 52;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_NEAR = 23;

    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double targetLauncherVelocity = 1;
    public static double LOWER_FEED_ARM_POSITION_1 = .6;
    public static double LOWER_FEED_ARM_POSITION_2 = .76;
    public static double LOWER_FEED_ARM_POSITION_3 = .85;
    public static double AXON_ENCODER_CO_EFF = 1;

    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height
    private final double LOWER_FEED_BAR_TOP_POSITION = .8;

    public static double SERVER_P_CO_EFF = .007;

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
        topLeftDistance = opMode.hardwareMap.get(DistanceSensor.class, "topLeftDistance");
        topRightDistance = opMode.hardwareMap.get(DistanceSensor.class, "topRightDistance");
        bottomLeftDistance = opMode.hardwareMap.get(DistanceSensor.class, "bottomLeftDistance");
        bottomRightDistance = opMode.hardwareMap.get(DistanceSensor.class, "bottomRightDistance");
        //serverPID = new MM_PID_CONTROLLER(SERVER_P_CO_EFF, 0, 0);

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
        opMode.multipleTelemetry.addData("serverSpeed", server.getPower());
        opMode.multipleTelemetry.addData("dist Left", topLeftDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Right", topRightDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Lower Left", bottomLeftDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Lower Right", bottomRightDistance.getDistance(DistanceUnit.MM));

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
        if (currentGamepad2.y){
            pusher.setPosition(0);
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
            double serverError = Math.abs(getAxonDegrees(serverEncoder) - serverStopPoint);
            if (serverError > 20) {
                serverIsReady = false;
                server.setPower(serverError * SERVER_P_CO_EFF);
            } else if (!serverIsReady) {
                serverIsReady = true;
                launching = false;
                server.setPower(0);
            }
        }
    }

    private void setTargetLauncherVelocity() {
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())); // unit is inches
        double metersPerSecond = Math.sqrt((9.81 * launchDistance * 0.0254) / Math.sin(LAUNCHER_ANGLE * 2));
        double ticksPerSecond = Math.abs(metersPerSecond * TICKS_PER_METER);

        if(launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_NEAR){
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_NEAR;
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_MID) {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_MID;
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_FIELD_CENTER) {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_FIELD_CENTER;
        } else {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_AUDIENCE;
        }

        opMode.multipleTelemetry.addData("launchDistance (inches)", launchDistance);
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
    }

    private boolean haveArtifactAtTop(){
        artifactAtTop = topLeftDistance.getDistance(DistanceUnit.MM) < 35 || topRightDistance.getDistance(DistanceUnit.MM) < 45;
        return true;
    }

    public boolean have3Artifacts(){
        return bottomLeftDistance.getDistance(DistanceUnit.MM) < 35 || bottomRightDistance.getDistance(DistanceUnit.MM) < 35;
    }

    public boolean lowerFeedArmReady(){
        return pusher.getPosition() == 0;
    }

    private double getAxonDegrees(AnalogInput encoder){
        return ((encoder.getVoltage() / 3.3) * 360);
    }

    public void setServerForInit(){
        double serverError = Math.abs(getAxonDegrees(serverEncoder) - serverStopPoint);

        if(!serverIsReady) {
            if (getAxonDegrees(serverEncoder) < serverStopPoint) {
                server.setPower(serverError * SERVER_P_CO_EFF);
            } else {
                serverIsReady = true;
                server.setPower(0);
            }
        }
    }

    public void updateProjectileTarget(){
        projectileTarget.setY(65 * alliance);
    }
}


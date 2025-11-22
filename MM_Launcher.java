package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

import com.acmerobotics.dashboard.config.Config;
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
    private ColorSensor peephole;
    private DistanceSensor peepholeDistance;
    private MM_Position projectileTarget = new MM_Position(-65, -65, 0); //blue goal pos

    public static double LAUNCHER_CO_EFF = 2.5; //2.3 for 30A wheels
    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double targetLauncherVelocity = 1;
    public static double LOWER_FEED_ARM_POSITION_1 = .26;
    public static double LOWER_FEED_ARM_POSITION_2 = .42;
    public static double LOWER_FEED_ARM_POSITION_3 = .54;

    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height
    private final double LOWER_FEED_BAR_TOP_POSITION = .8;
    public static double HYPOT_SPEED_THRESHOLD = 23;
    public static double HYPOT_SPEED_CO_EFF = 1.5;


    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    private boolean artifactAtTop = true;
    private boolean serverIsReady = true;
    private boolean launching = false;


    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;

        launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
        launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
        launchMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        pusher = opMode.hardwareMap.get(Servo.class, "launcherFeeder");
        pusher.setPosition(0);
        server = opMode.hardwareMap.get(CRServo.class, "upperFeedArm");
        peephole = opMode.hardwareMap.get(ColorSensor.class, "upperFeedSensor");
        peepholeDistance = opMode.hardwareMap.get(DistanceSensor.class, "upperFeedSensor");
        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLauncher() {
        setTargetLauncherVelocity();

        opMode.multipleTelemetry.addData("launcherTargetSpeed", targetLauncherVelocity);
        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());

        //if (launching){
        if(currentGamepad1.b && !previousGamepad1.b) {

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

        if (haveArtifact() || launching) { //TODO only set velocity once
            launchMotorLeft.setVelocity(targetLauncherVelocity);
            launchMotorRight.setVelocity(targetLauncherVelocity);
        } else {
            launchMotorLeft.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
            launchMotorRight.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
        }

        if (pusher.getPosition() >= LOWER_FEED_ARM_POSITION_1 && !launching && artifactAtTop && currentGamepad1.right_trigger > 0  && Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50) {
//            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION); TODO fix the lower feed arm
            server.setPower(1);
            launching = true;
        }
        opMode.multipleTelemetry.addData("colors", "red %d, green %d, blue %d", peephole.red(), peephole.green(), peephole.blue());
        opMode.multipleTelemetry.addData("Distance", peepholeDistance.getDistance(DistanceUnit.MM));

        if(launching) {
            if (peephole.red() < 350) {
                serverIsReady = false;
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

    private boolean haveArtifact(){
        return artifactAtTop;
    }

    public boolean lowerFeedArmReady(){
        return pusher.getPosition() == 0;
    }

}


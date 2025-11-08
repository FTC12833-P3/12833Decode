package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MM_Launcher {
    private MM_OpMode opMode;
    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo lowerFeedArm;
    private CRServo mo;
    private ColorSensor peephole;
    private MM_Position projectileTarget = new MM_Position(-65, -65, 0); //blue goal pos

    public static double LAUNCHER_CO_EFF = 1.76; //2.3 for 30A wheels
    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double targetLauncherVelocity = 1;
    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height
    private final double LOWER_FEED_BAR_TOP_POSITION = .8;

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    private boolean artifactAtTop = true;
    private boolean moIsReady = true;
    private boolean launching = false;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;

        launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
        launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
        launchMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        lowerFeedArm = opMode.hardwareMap.get(Servo.class, "launcherFeeder");
        mo = opMode.hardwareMap.get(CRServo.class, "upperFeedArm");
        peephole = opMode.hardwareMap.get(ColorSensor.class, "upperFeedSensor");
        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLauncher() {
        setTargetLauncherVelocity();

        opMode.multipleTelemetry.addData("launcherTargetSpeed", targetLauncherVelocity);
        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());

        if (haveArtifact() || launching) { //TODO only set velocity once
            launchMotorLeft.setVelocity(targetLauncherVelocity);
            launchMotorRight.setVelocity(targetLauncherVelocity);
        } else {
            launchMotorLeft.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
            launchMotorRight.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
        }

        if (!launching && artifactAtTop && currentGamepad2.right_trigger > 0  && Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50) {
//            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION); TODO fix the lower feed arm
            mo.setPower(1);
            launching = true;
        }

        if(launching) {
            if (peephole.red() < 1000) {
                moIsReady = false;
            } else if (!moIsReady) {
                moIsReady = true;
                launching = false;
                mo.setPower(0);
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
    }

    private boolean haveArtifact(){
        return artifactAtTop;
    }

}


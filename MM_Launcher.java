package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MM_Launcher {
    private MM_OpMode opMode;
    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo lowerFeedArm;
    private CRServo upperFeedArm;
    private MM_Position projectileTarget = new MM_Position(-65, -65, 0); //blue goal pos

    public static double LAUNCHER_CO_EFF = 1.76; //2.3 for 30A wheels
    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double launcherSpeed = 1;
    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height
    private final double LOWER_FEED_BAR_TOP_POSITION = .8;

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    private boolean haveArtifact = true;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;

        launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
        launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
        launchMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        lowerFeedArm = opMode.hardwareMap.get(Servo.class, "launcherFeeder");
        upperFeedArm = opMode.hardwareMap.get(CRServo.class, "upperFeedArm");
        launchMotorLeft.setTargetPosition(10);
        launchMotorRight.setTargetPosition(10);
        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLauncher() {
        calculateLauncherVelocity();
        opMode.multipleTelemetry.addData("launcherTargetSpeed", launcherSpeed);
        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());

        if (haveArtifact) {
            launchMotorLeft.setVelocity(launcherSpeed);
            launchMotorRight.setVelocity(launcherSpeed);
        } else {
            launchMotorLeft.setVelocity(launcherSpeed * SLOW_SPEED_CO_EFF);
            launchMotorRight.setVelocity(launcherSpeed * SLOW_SPEED_CO_EFF);
        }

        if (haveArtifact && currentGamepad2.left_trigger > 0 && launchMotorLeft.getVelocity() - launcherSpeed < 50) {
            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION);
            upperFeedArm.setPower(1);
        } else {
            upperFeedArm.setPower(0);
        }
    }

    private void calculateLauncherVelocity() {
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())) * 0.0254;
        opMode.multipleTelemetry.addData("launchDistance", launchDistance);
        //double metersPerSecond = Math.sqrt(Math.abs((launchDistance * Math.tan(LAUNCHER_ANGLE) - (9.81 * Math.pow(launchDistance, 2)) )/ (2 * FINAL_PROJECTILE_HEIGHT * Math.pow(Math.cos(LAUNCHER_ANGLE), 2))));
        double metersPerSecond = Math.sqrt((9.81 * launchDistance) / Math.sin(LAUNCHER_ANGLE * 2));
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
        launcherSpeed = Math.abs(metersPerSecond * TICKS_PER_METER) * LAUNCHER_CO_EFF;
    }

}

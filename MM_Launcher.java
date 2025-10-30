package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Launcher {
    private MM_OpMode opMode;
    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo launcherFeeder;
    private MM_Position projectileTarget = new MM_Position(-65, -65, 0); //blue goal pos

    private double LAUNCHER_ANGLE = 45;
    public static boolean runLauncher = false;
    public static double launcherSpeed = 1;
    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 75; //mm
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;



    public MM_Launcher(MM_OpMode opMode){
     this.opMode = opMode;

     launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
     launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
     launchMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
     launcherFeeder = opMode.hardwareMap.get(Servo.class, "launcherFeeder");
        launchMotorLeft.setTargetPosition(10);
        launchMotorRight.setTargetPosition(10);
        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void runLauncher(){
//        calculateLauncherVelocity();
//        opMode.multipleTelemetry.addData("launcherSpeed", launcherSpeed);
//        opMode.multipleTelemetry.addData("launcherRealSpeed", launchMotorLeft.getVelocity());
//        launchMotorLeft.setVelocity(launcherSpeed);
//        launchMotorRight.setVelocity(launcherSpeed);

    }

    private void calculateLauncherVelocity(){
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                            projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())) * 0.0254;
        opMode.multipleTelemetry.addData("launchDistance", launchDistance);
        double metersPerSecond = Math.sqrt(Math.abs((launchDistance * Math.tan(LAUNCHER_ANGLE) - (9.81 * Math.pow(launchDistance, 2)) )/ (2 * FINAL_PROJECTILE_HEIGHT * Math.pow(Math.cos(LAUNCHER_ANGLE), 2))));
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
        launcherSpeed = Math.abs(metersPerSecond * TICKS_PER_METER);
    }

}

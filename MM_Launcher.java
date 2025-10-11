package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Launcher {
    private MM_OpMode opMode;
    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo launcherFeeder;
    private MM_Position projectileTarget = new MM_Position(-65, -65, 0); //blue goal pos

    private double launcherAngle = 45;
    public static boolean runLauncher = false;
    public static double launcherSpeed = 1;
    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height


    public MM_Launcher(MM_OpMode opMode){
     this.opMode = opMode;

     launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
     launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
     launcherFeeder = opMode.hardwareMap.get(Servo.class, "launcherFeeder");
    }

    public void runLauncher(){
        calculateLauncherVelocity();
        if (runLauncher){
            launchMotorLeft.setVelocity(launcherSpeed);
            launchMotorRight.setVelocity(launcherSpeed);
        } else { //TODO remove else
            launchMotorLeft.setVelocity(0);
            launchMotorRight.setVelocity(0);
        }

    }

    private void calculateLauncherVelocity(){
        double launchDistance = (Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                            projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY()));
        launcherSpeed = Math.sqrt(launchDistance * Math.tan(launcherAngle) - (9.81 * Math.pow(launchDistance, 2)) / (2 * FINAL_PROJECTILE_HEIGHT * Math.pow(Math.cos(launcherAngle), 2)));
    }

}

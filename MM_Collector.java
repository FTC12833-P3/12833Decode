package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MM_Collector {
    MM_OpMode opMode;
    DcMotorEx collector;

    private final double COLLECT_POWER = 1;
    public static boolean runCollector = false;
    public static boolean reverseCollector = false;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotorEx.class, "collector");
    }

    public void runCollector(){
        if(opMode.gamepad2.right_bumper && opMode.robot.launcher.lowerFeedArmReady() && !opMode.robot.launcher.lowerSensorTriggered()){
            collector.setPower(COLLECT_POWER);
        } else if (opMode.gamepad2.left_bumper){
            collector.setPower(-COLLECT_POWER);
        } else {
            collector.setPower(0);
        }
    }
    
    public void autoRunCollector(){
        if(runCollector && !opMode.robot.launcher.lowerSensorTriggered()){
            collector.setPower(COLLECT_POWER);
        } else if (reverseCollector){
            collector.setPower(-COLLECT_POWER);
        } else {
            collector.setPower(0);
        }
    }
}

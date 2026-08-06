package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MM_Collector {
    MM_OpMode opMode;
    DcMotorEx collector;

    private static double collectPower = 0.775;
    public static boolean runCollector = false;
    public static boolean reverseCollector = false;

    MM_Collector(MM_OpMode opMode){
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotorEx.class, "collector");
        collectPower =  opMode.getClass() == MM_Autos.class? 1: collectPower;
    }

    public void runCollector(){
        if(opMode.gamepad2.right_bumper && opMode.robot.launcher.lowerFeedArmReady() && opMode.robot.launcher.serverReadyForCollect() && (!opMode.robot.launcher.lowerSensorTriggered() || opMode.gamepad2.a)){
            collector.setPower(collectPower);
        } else if (opMode.gamepad2.left_bumper){
            collector.setPower(-collectPower);
        } else {
            collector.setPower(0);
        }

//        if(currentGamepad2.a && !previousGamepad2.a){
//            COLLECT_POWER = COLLECT_POWER == 1? .2: 1;
//        }
    }
    
    public void autoRunCollector(){
        if(runCollector){ //&& !opMode.robot.launcher.lowerSensorTriggered
            collector.setPower(collectPower);
        } else if (reverseCollector){
            collector.setPower(-collectPower);
        } else {
            //collector.setPower(0);
        }
    }
}

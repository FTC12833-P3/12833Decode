package org.firstinspires.ftc.teamcode;

public class MM_Robot {
    private MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Launcher launcher;

    public MM_Robot (MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        launcher = new MM_Launcher(opMode);
    }
}

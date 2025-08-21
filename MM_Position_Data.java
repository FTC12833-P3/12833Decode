package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class MM_Position_Data {
    public static final double TAG_FLIP_THRESHOLD = .2;
    private final MM_OpMode opMode;

    public MM_VisionPortal visionPortal;
    public GoBildaPinpointDriver odometryController;

    private Pose2D currentPos;
    private Pose2D AprilTagPos;
    public double pastExtrinsicY;

    public static MM_Position targetPos = new MM_Position(0, 0, 0);
    public MM_Spline testSpline = new MM_Spline(new double[]{36, 60, 36}, new double[]{24, 0, -24}, MM_Autos.SPLINE_DETAIL_LEVEL);
    public MM_Spline testCubicSpline = new MM_Spline(new double[]{36, 96, 0, 48}, new double[]{46, 0, 0, -48}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline continuousCubicSpline = new MM_Spline(new double[]{36,104, 0, 48}, new double[]{49, 24, 24, 0}, MM_Autos.SPLINE_DETAIL_LEVEL, true);

    //xPoints 48, 96, 0, 48
    //yPoints , 0, -24, -24, -48
    MM_Position_Data(MM_OpMode opMode){
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);
        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometryController.setOffsets(53.975, 3.175);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if(opMode.getClass() == MM_Autos.class) {
            odometryController.resetPosAndIMU();
        }

        odometryController.update();
        currentPos = odometryController.getPosition();
        targetPos.setAll(0, 0, 0);
    }

    public void updatePosition(){
        updatePosition(false);
    }

    public void updatePosition(boolean useApriltag){
        currentPos = odometryController.getUpdatedPositon();

        opMode.multipleTelemetry.addData("xOdom", round2Dec(getX()));
        opMode.multipleTelemetry.addData("yOdom", round2Dec(getY()));
        opMode.multipleTelemetry.addData("yawOdom", round2Dec(getHeading()));

        if (useApriltag) {
            if (AprilTagPos != null) {
                pastExtrinsicY = AprilTagPos.getY(DistanceUnit.INCH);
            }
            AprilTagPos = visionPortal.setPosFromApriltag();
            if (AprilTagPos != null) {
                opMode.multipleTelemetry.addData("xApril", round2Dec(AprilTagPos.getX(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yApril", round2Dec(AprilTagPos.getY(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yawApril", round2Dec(AprilTagPos.getHeading(AngleUnit.DEGREES)));

                if ((opMode.opModeInInit() && MM_OpMode.currentGamepad1.b && !MM_OpMode.previousGamepad1.b) || (!opMode.opModeInInit() && opMode.getClass() != MM_Autos.class)) {
                    odometryController.setPosition(AprilTagPos);
                    if(opMode.opModeInInit()){
                        MM_OpMode.alliance = MM_VisionPortal.startingTag == 13 || MM_VisionPortal.startingTag == 11? -1: 1; //TODO adjust for correct tags
                        MM_OpMode.scoringLocation = MM_VisionPortal.startingTag == 13 || MM_VisionPortal.startingTag == 16? "": "";
                    }
                }
            } else { //just here for dashboard
                opMode.multipleTelemetry.addData("xApril", "");
                opMode.multipleTelemetry.addData("yApril", "");
                opMode.multipleTelemetry.addData("yawApril", "");
            }
        }

        currentPos = odometryController.getUpdatedPositon();
    }


        private double round2Dec ( double inDouble){
        return Math.round(inDouble * 100) / 100.0;
    }

        public double getX () {
        return currentPos.getX(DistanceUnit.INCH);
    }


        public double getY () {
        return currentPos.getY(DistanceUnit.INCH);
    }

        public double getHeading () {
        return currentPos.getHeading(AngleUnit.DEGREES);
    }

        public void setPosition ( double xPos, double yPos, double yawPos){
        odometryController.setPosition(new Pose2D(DistanceUnit.INCH, xPos, yPos, AngleUnit.DEGREES, yawPos));
    }

}
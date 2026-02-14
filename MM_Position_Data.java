package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

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

    private static Pose2D currentPos;
    private Pose2D AprilTagPos;
    public double pastExtrinsicY;

    public double firstSpikeX = -15;
    public double thirdSpikeX = 32;
    public double gateEndingControlPointsX = -7.5;
    public static MM_Position targetPos = new MM_Position(0, 0, 0);

    public MM_Spline goalSideSplineToCollectSecondSpikeMark = new MM_Spline(new double[]{-20, -1.4, 8, 8}, new double[]{20, 17.7, 20.3, 33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline audienceSideSplineToCollectSecondSpikeMark = new MM_Spline(new double[]{53, 31, 8, 8}, new double[]{17, 17, 15, 33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline audienceSideSplineToCollectThirdSpikeMark = new MM_Spline(new double[]{53, 12, -13, -15}, new double[]{17, 17, 15, 33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);

    public MM_Spline goalSideSplineToCollectThirdSpikeMark = new MM_Spline(new double[]{-20, 15, 32, 32}, new double[]{20, 20, 18, 33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline goalSideSplineToOpenGate = new MM_Spline(new double[]{8, 8, gateEndingControlPointsX, gateEndingControlPointsX}, new double[]{56, 42, 42, 56}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline audienceSideSplineToOpenGate = new MM_Spline(new double[]{-15, -15, gateEndingControlPointsX, gateEndingControlPointsX}, new double[]{56, 42, 42, 56}, MM_Autos.SPLINE_DETAIL_LEVEL, true);

    MM_Position_Data(MM_OpMode opMode) {
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);
        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometryController.setOffsets(19.275, 173.475);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (opMode.getClass() == MM_Autos.class) {
            odometryController.resetPosAndIMU();
        } else {
            try{
                odometryController.setPosition(currentPos);
            } catch (Exception NullPointer){
                odometryController.resetPosAndIMU();
            }


        }

        odometryController.update();
        currentPos = odometryController.getPosition();
        targetPos.setAll(0, 0, 0);
    }
    public void initSpikeMarks(){
        odometryController.update();
        currentPos = odometryController.getPosition();
        if(opMode.gamepad1.dpad_left && currentGamepad1.b && !previousGamepad1.b){
            firstSpikeX = currentPos.getX(DistanceUnit.INCH);
        }
        if(opMode.gamepad1.dpad_up && currentGamepad1.b && !previousGamepad1.b){
            opMode.chosenSplineList.get(1).setLastPoint(currentPos.getX(DistanceUnit.INCH));
        }
        if(opMode.gamepad1.dpad_right && currentGamepad1.b && !previousGamepad1.b){
            opMode.chosenSplineList.get(2).setLastPoint(currentPos.getX(DistanceUnit.INCH));
        }
        if(opMode.gamepad1.dpad_right && currentGamepad1.y && !previousGamepad1.y){
            opMode.multipleTelemetry.addData("lockingInToGate", "in progress...");
            opMode.multipleTelemetry.update();
            gateEndingControlPointsX = currentPos.getX(DistanceUnit.INCH);
            goalSideSplineToOpenGate = new MM_Spline(new double[]{-15, -15, gateEndingControlPointsX, gateEndingControlPointsX}, new double[]{56, 42, 42, 56}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
            opMode.multipleTelemetry.addData("lockingInToGate", "lockedIn");
            opMode.multipleTelemetry.update();
        }
        opMode.multipleTelemetry.addData("firstSpikeX", firstSpikeX);
        opMode.multipleTelemetry.addData("secondSpikeX", goalSideSplineToCollectSecondSpikeMark.getxPoints()[MM_Autos.SPLINE_DETAIL_LEVEL]);
        opMode.multipleTelemetry.addData("gatePos", gateEndingControlPointsX);
        opMode.multipleTelemetry.addData("thirdSpikeX", goalSideSplineToCollectThirdSpikeMark.getxPoints()[MM_Autos.SPLINE_DETAIL_LEVEL]);

    }


    public void updatePosition() {
        currentPos = odometryController.getUpdatedPositon();

        opMode.multipleTelemetry.addData("xOdom", round2Dec(getX()));
        opMode.multipleTelemetry.addData("yOdom", round2Dec(getY()));
        opMode.multipleTelemetry.addData("yawOdom", round2Dec(getHeading()));

        if(currentGamepad1.dpad_down) {
            AprilTagPos = visionPortal.setPosFromApriltag();
            if (AprilTagPos != null) {
                opMode.multipleTelemetry.addData("xApril", round2Dec(AprilTagPos.getX(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yApril", round2Dec(AprilTagPos.getY(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yawApril", round2Dec(AprilTagPos.getHeading(AngleUnit.DEGREES)));

                if (currentGamepad1.b && !MM_OpMode.previousGamepad1.b) {
                    odometryController.setPosition(AprilTagPos);
                    if (opMode.opModeInInit()) {
                        MM_OpMode.alliance = MM_VisionPortal.startingTag == 20 ? -1 : 1; //blue = -1
                    }
                    opMode.robot.launcher.updateProjectileTarget();
                }
            } else { //just here for dashboard
                opMode.multipleTelemetry.addData("xApril", "");
                opMode.multipleTelemetry.addData("yApril", "");
                opMode.multipleTelemetry.addData("yawApril", "");
            }
        }
        currentPos = odometryController.getUpdatedPositon();
    }


    private double round2Dec(double inDouble) {
        return Math.round(inDouble * 100) / 100.0;
    }

    public double getX() {
        return currentPos.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return currentPos.getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return currentPos.getHeading(AngleUnit.DEGREES);
    }

    public void setPosition(double xPos, double yPos, double yawPos) {
        odometryController.setPosition(new Pose2D(DistanceUnit.INCH, xPos, yPos, AngleUnit.DEGREES, yawPos));
    }

}
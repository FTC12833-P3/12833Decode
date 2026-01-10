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
    public MM_Spline splineToCollectFirstSpikeMark = new MM_Spline(new double[]{-20, -1.4, 14.7, 9}, new double[]{20 * MM_OpMode.alliance, 17.7 * MM_OpMode.alliance, 20.3 * MM_OpMode.alliance, 33 * MM_OpMode.alliance}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline splineToCollectSecondSpikeMark = new MM_Spline(new double[]{-20, -11, -1, 18}, new double[]{-20, -13, -16, -33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);
    public MM_Spline splineToCollectThirdSpikeMark = new MM_Spline(new double[]{-12, 15, 36, 36}, new double[]{-12, -16, -16, -33}, MM_Autos.SPLINE_DETAIL_LEVEL, true);

    MM_Position_Data(MM_OpMode opMode) {
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);
        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometryController.setOffsets(19.275, 173.475);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (opMode.getClass() == MM_Autos.class) {
            odometryController.resetPosAndIMU();
        }
        odometryController.resetPosAndIMU(); //TODO remove this


        odometryController.update();
        currentPos = odometryController.getPosition();
        targetPos.setAll(0, 0, 0);
    }


    public void updatePosition() {
        currentPos = odometryController.getUpdatedPositon();

        opMode.multipleTelemetry.addData("xOdom", round2Dec(getX()));
        opMode.multipleTelemetry.addData("yOdom", round2Dec(getY()));
        opMode.multipleTelemetry.addData("yawOdom", round2Dec(getHeading()));

        if(MM_OpMode.currentGamepad1.dpad_down) {
            AprilTagPos = visionPortal.setPosFromApriltag();
            if (AprilTagPos != null) {
                opMode.multipleTelemetry.addData("xApril", round2Dec(AprilTagPos.getX(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yApril", round2Dec(AprilTagPos.getY(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yawApril", round2Dec(AprilTagPos.getHeading(AngleUnit.DEGREES)));

                if (MM_OpMode.currentGamepad1.b && !MM_OpMode.previousGamepad1.b) {
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
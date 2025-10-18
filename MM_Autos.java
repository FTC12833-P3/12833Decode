package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "autos", group = "MM")
public class MM_Autos extends MM_OpMode {
    public static int SPLINE_DETAIL_LEVEL = 20;

    private enum STATES {
        SCORE,
        DRIVE_TO_COLLECT,
        COLLECT,
        DRIVE_TO_LAUNCH,
        LAUNCH
    }

    STATES previousState = null;

    int currentSection = 0;
    double targetX;
    double targetY;
    double heading;
    int collectCycle = -1; //negative one to account for the first score state
    boolean notDone = true;
    boolean readyToCollect = false;
    List<MM_Spline> collectSplines;

    private STATES state = STATES.SCORE;

    @Override
    public void runProcedures() {

        collectSplines = Arrays.asList(robot.drivetrain.navigation.splineToCollectFirstSpikeMark, robot.drivetrain.navigation.splineToCollectSecondSpikeMark, robot.drivetrain.navigation.splineToCollectThirdSpikeMark);
        robot.drivetrain.enableBrakes();

        while (opModeIsActive() && notDone) {
            switch (state) {
                case SCORE:
                    //TODO launch
                    if (state != previousState) {
                        previousState = state;
                        if (collectCycle < 2) {
                            MM_Position_Data.targetPos.setAll(-12, -12, 45);
                        } else {
                            MM_Position_Data.targetPos.setAll(54, -12, 23);
                        }
                    } else if (robot.drivetrain.driveDone()) {
                        previousState = state;
                        state = STATES.DRIVE_TO_COLLECT;
                        if (collectCycle >= collectSplines.size() - 1) {
                            notDone = false;
                        }

                        collectCycle++;

                    }
                    break;
                case DRIVE_TO_COLLECT:
                    if (state != previousState) {
                        if (collectCycle != 1) {
                            if (collectCycle < collectSplines.size()) {
                                previousState = state;
                                heading = -90;
                                prepareToSpline(collectSplines.get(collectCycle));
                            }
                        } else {
                                MM_Position_Data.targetPos.setAll(12, -33, -90);
                        }
                    }

                    if (robot.drivetrain.driveDone()) {
                        if (currentSpline != null) {
                            setNextSplinePoint(currentSpline);
                            multipleTelemetry.addLine("not following spline");
                        }
                        readyToCollect = true;
                    }
                    multipleTelemetry.addData("currentTargetX", MM_Position_Data.targetPos.getX());

                    if (splineDone() || (readyToCollect && collectCycle == 1)){
                        previousState = state;
                        state = STATES.COLLECT;
                        readyToCollect = false;
                    }
                    break;
                case COLLECT:
                    if (state != previousState) {
                        MM_Collector.runCollector = true;
                        MM_Position_Data.targetPos.setY(MM_Position_Data.targetPos.getY() - 15);
                        previousState = state;

                    }


                    if (robot.drivetrain.driveDone()) {
                        MM_Collector.runCollector = false;
                        state = STATES.SCORE;
                    }

            }
            robot.drivetrain.autoRunDrivetrain();
            robot.collector.autoRunCollector();
            multipleTelemetry.update();

        }
    }


    public void setNextSplinePoint(MM_Spline spline) {

        spline.updateDistanceTraveled(currentSection);
        targetX = spline.getNextPoint(currentSection)[0];
        targetY = spline.getNextPoint(currentSection)[1];
        MM_Position_Data.targetPos.setAll(targetX, targetY, heading);
        currentSection++;
    }

    public void prepareToSpline(MM_Spline spline) {
        spline.resetDistanceTraveled();
        currentSpline = spline;
        setNextSplinePoint(spline);
        MM_Drivetrain.xErrorThreshold = 4;
        MM_Drivetrain.yErrorThreshold = 4;
        MM_Drivetrain.headingErrorThreshold = 360;
        MM_PID_CONTROLLER.D_COEFF = 0;
        currentSection -= 1;
    }

    public boolean splineDone() {
        if (currentSection == MM_Autos.SPLINE_DETAIL_LEVEL + 1) {
            MM_Drivetrain.xErrorThreshold = MM_Drivetrain.X_ERROR_THRESHOLD;
            MM_Drivetrain.yErrorThreshold = MM_Drivetrain.Y_ERROR_THRESHOLD;
            MM_Drivetrain.headingErrorThreshold = MM_Drivetrain.HEADING_ERROR_THRESHOLD;
            MM_PID_CONTROLLER.D_COEFF = 30;
            currentSection = 0;
            currentSpline = null;
            return true;
        }
        return false;
    }
}

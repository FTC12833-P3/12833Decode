package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "autos", group = "MM")
public class MM_Autos extends MM_OpMode {
    public static final int REGULAR_DRIVE_D_CO_EFF = 30;
    public static int SPLINE_DETAIL_LEVEL = 20;

    private enum STATES {
        DRIVE_TO_SCORE,
        SCORE,
        DRIVE_TO_COLLECT,
        COLLECT,
        LOOK_AT_MOTIF
    }

    STATES previousState = null;

    int currentSection = 0;
    double targetX;
    double targetY;
    double heading;
    int collectCycle = -1; //negative one to account for the first score state
    boolean notDone = true;
    boolean motifDone = false;
    boolean rotateDone = true;
    List<MM_Spline> collectSplines;

    private STATES state = STATES.DRIVE_TO_SCORE;

    @Override
    public void runProcedures() {

        collectSplines = Arrays.asList(robot.drivetrain.navigation.splineToCollectFirstSpikeMark, robot.drivetrain.navigation.splineToCollectSecondSpikeMark);
        robot.drivetrain.enableBrakes();

        while (opModeIsActive() && notDone) {
            switch (state) {
                case DRIVE_TO_SCORE:
                    if (state != previousState) {
                        previousState = state;
                        MM_Position_Data.targetPos.setAll(-20, 20 * alliance, 135 * alliance);

                    } else if (robot.drivetrain.driveDone()) {
                        state = STATES.SCORE;
                        break;
                    }
                    break;
                case SCORE:
                    if (state != previousState) {
                        previousState = state;
                        MM_Launcher.scoreArtifacts = true;
                    } else if (!MM_Launcher.scoreArtifacts) {
                        state = STATES.DRIVE_TO_COLLECT;
                        if (collectCycle >= 1) {
                            notDone = false;
                        }
                        if(!eliminationMatch){
                            if (motif == -1) {
                                state = STATES.LOOK_AT_MOTIF;
                            } else if (!allSpikes){
                                if(spike1 && motif != 0 && collectCycle < 0){
                                    collectCycle = 0;
                                } else if (spike2 && motif != 1 && collectCycle < 1){
                                    collectCycle = 1;
                                } else if (spike3 && motif != 2 && collectCycle < 2){
                                    collectCycle = 2;
                                } else {
                                    notDone = false;
                                }
                            }
                        } else {
                            if(allSpikes) {
                                collectCycle++;
                            } else if (spike1 && collectCycle < 0){
                                collectCycle = 0;
                            } else if (spike2 && collectCycle < 1){
                                collectCycle = 1;
                            } else if (collectCycle < 2){
                                collectCycle = 2;
                            }
                        }
                    }
                    break;
                case LOOK_AT_MOTIF:
                    if(state != previousState){
                        previousState = state;
                        MM_Position_Data.targetPos.setHeading(150 * alliance);
                    }

                    if(robot.drivetrain.driveDone()){
                        robot.drivetrain.navigation.visionPortal.setMotif();
                        if(motif != -2) {
                            collectCycle = Math.abs(motif - 2);
                        }
                        state = STATES.COLLECT;
                    }

                    break;
                case DRIVE_TO_COLLECT:
                    if (state != previousState) {
                        MM_Position_Data.targetPos.setHeading(-90 * alliance);
                        if(collectCycle == 0) {
                            rotateDone = false;
                        } else {
                            rotateDone = true;
                        }
                        previousState = state;
                        prepareToSpline(collectSplines.get(collectCycle));
                    }

                    if (robot.drivetrain.driveDone() && rotateDone) {
                        previousState = state;
                        if(collectCycle == 0 || splineDone()) {
                            state = STATES.COLLECT;
                        } else {
                            setNextSplinePoint(currentSpline);

                        }
                    } else if (robot.drivetrain.driveDone() && collectCycle == 0){
                        MM_Position_Data.targetPos.setAll(-15, 33 * alliance, -90 * alliance);
                        rotateDone = true;
                    }
                    multipleTelemetry.addData("currentCycle", collectCycle);
                    multipleTelemetry.addData("currentTargetX", MM_Position_Data.targetPos.getX());
                    MM_Collector.runCollector = true;

                    break;
                case COLLECT:

                    MM_Position_Data.targetPos.setY(56 * alliance);

                    if (state != previousState) {
                        previousState = state;
                    } else if (robot.drivetrain.driveDone()) {
                        if(!motifDone){
                            motifDone = true;
                            collectCycle = -1;
                        }
                        state = STATES.DRIVE_TO_SCORE;
                    }

            }
            robot.drivetrain.autoRunDrivetrain();
            robot.collector.autoRunCollector();
            robot.launcher.autoRunLauncher();

            multipleTelemetry.addData("dCoeff", MM_Drivetrain.DrivePidController.getD_COEFF());
            multipleTelemetry.update();
        }
    }

    public void setNextSplinePoint(MM_Spline spline) {
        spline.updateDistanceTraveled(currentSection);
        targetX = spline.getNextPoint(currentSection)[0];
        targetY = spline.getNextPoint(currentSection)[1];
        MM_Position_Data.targetPos.setAll(targetX, -targetY * alliance, -heading * alliance);
        currentSection++;
    }

    public void prepareToSpline(MM_Spline spline) {
        spline.resetDistanceTraveled();
        currentSpline = spline;
        setNextSplinePoint(spline);
        MM_Drivetrain.xErrorThreshold = 4;
        MM_Drivetrain.yErrorThreshold = 4;
        MM_Drivetrain.DrivePidController.setD_COEFF(0);
        currentSection -= 1;
    }

    public boolean splineDone() {
        if (currentSection == MM_Autos.SPLINE_DETAIL_LEVEL + 1) {
            MM_Drivetrain.xErrorThreshold = MM_Drivetrain.X_ERROR_THRESHOLD;
            MM_Drivetrain.yErrorThreshold = MM_Drivetrain.Y_ERROR_THRESHOLD;
            MM_Drivetrain.DrivePidController.setD_COEFF(REGULAR_DRIVE_D_CO_EFF);

            currentSection = 0;
            currentSpline = null;
            return true;
        }
        return false;
    }
}

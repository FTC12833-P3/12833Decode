package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "autos", group = "MM")
public class MM_Autos extends MM_OpMode {
    public static final int REGULAR_DRIVE_D_CO_EFF = 30;
    public static final int SPLINE_DETAIL_LEVEL = 20; //TODO test different values

    private enum STATES {
        DRIVE_TO_SCORE,
        SCORE,
        DRIVE_TO_COLLECT,
        COLLECT,
        LOOK_AT_MOTIF,
        DRIVE_OFF_LINE,
        OPEN_GATE,
    }

    STATES previousState = null;

    int currentSection = 0;
    double targetX;
    double targetY;
    double targetHeading;
    double finalSpike = 0;
    int collectCycle = -1; //negative one to account for the first score state
    boolean notDone = true;
    boolean motifDone = false;
    boolean rotateDone = true;
    boolean lastCycle = false;
    boolean timerStarted = false;
    boolean spike1Done = false;
    boolean spike2Done = false;
    ElapsedTime gateTime = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();

    private STATES state = STATES.DRIVE_TO_SCORE;

    @Override
    public void runProcedures() {
        goalSideCollectSplines = Arrays.asList(null, robot.drivetrain.navigation.goalSideSplineToCollectSecondSpikeMark, robot.drivetrain.navigation.goalSideSplineToCollectThirdSpikeMark, robot.drivetrain.navigation.goalSideSplineToOpenGate);
        audienceSideCollectSplines = Arrays.asList(null, robot.drivetrain.navigation.audienceSideSplineToCollectSecondSpikeMark, robot.drivetrain.navigation.audienceSideSplineToCollectThirdSpikeMark, robot.drivetrain.navigation.audienceSideSplineToOpenGate);

        chosenSplineList = settings[SETTINGS.GOAL_SIDE.ordinal()] == 1 ? goalSideCollectSplines: audienceSideCollectSplines;

        robot.drivetrain.enableBrakes();

        if(settings[SETTINGS.HP_COLLECT.ordinal()] == 1 && settings[SETTINGS.GOAL_SIDE.ordinal()] == 0){
            finalSpike = 20; //we want to try for as many as possible
        } else if(settings[SETTINGS.ALL_SPIKES.ordinal()] != 0 || settings[SETTINGS.SPIKE_3.ordinal()] != 0){
            finalSpike = 2;
        } else if (settings[SETTINGS.SPIKE_2.ordinal()] != 0){
            finalSpike = 1;
        } else if (settings[SETTINGS.SPIKE_1.ordinal()] != 0){
            finalSpike = 0;
        }
        gameTime.reset();

        while (opModeIsActive()) {
            robot.drivetrain.navigation.odometryController.getUpdatedPositon();
            robot.drivetrain.navigation.updatePosition(); //double checked that both of these lines are needed

            if(notDone && gameTime.milliseconds() >= 25000){
                lastCycle = true;
            }
            if(notDone) {
                switch (state) {
                    case DRIVE_TO_SCORE:
                        if (state != previousState) {
                            previousState = state;
                            if(collectCycle == -1 ||(spike1Done && spike2Done) || chosenSplineList.get(collectCycle) == null) {
                                if (settings[SETTINGS.GOAL_SIDE.ordinal()] == 1) {
                                    MM_Position_Data.targetPos.setAll(-20, 20 * alliance, alliance == BLUE ? -138.5 : 133);
                                } else {
                                    MM_Position_Data.targetPos.setAll(53, 17 * alliance, 158.2 * alliance);
                                }
                            } else {
                                prepareToSpline(chosenSplineList.get(collectCycle), true);
                                if(settings[SETTINGS.GOAL_SIDE.ordinal()] == 1){
                                    MM_Position_Data.targetPos.setHeading(alliance == BLUE ? -138.5 : 133);
                                    targetHeading = alliance == BLUE ? -138.5 : 133;
                                } else {
                                    MM_Position_Data.targetPos.setHeading(158.2 * alliance);
                                    targetHeading = 158.2 * alliance;
                                }

                            }
                            driveTime.reset();
                        } else if (robot.drivetrain.driveDone()) {
                            if(splineDone(true)){
                                MM_Collector.runCollector = true;
                                state = STATES.SCORE;
                                break;
                            } else {
                                setNextSplinePoint(currentSpline, true);
                                if(settings[SETTINGS.GOAL_SIDE.ordinal()] == 1){
                                    MM_Position_Data.targetPos.setHeading(alliance == BLUE ? -138.5 : 133);
                                    targetHeading = alliance == BLUE ? -138.5 : 133;
                                } else {
                                    MM_Position_Data.targetPos.setHeading(158.2 * alliance);
                                    targetHeading = 158.2 * alliance;
                                }
                            }

                        }
                        break;
                    case SCORE:
                        if (state != previousState) {
                            previousState = state;
                            MM_Launcher.scoreArtifacts = true;
                        } else if (!MM_Launcher.scoreArtifacts) {
                            state = STATES.DRIVE_TO_COLLECT;
                            if (collectCycle >= finalSpike) {
                                lastCycle = true;
                            }
                            if (settings[SETTINGS.ELIMINATION_MATCH.ordinal()] == 0) {
                                if (motif == -1) {
                                    state = STATES.LOOK_AT_MOTIF;
                                } else if (settings[SETTINGS.ALL_SPIKES.ordinal()] == 0) {
                                    if (settings[SETTINGS.SPIKE_1.ordinal()] == 1 && motif != 0 && collectCycle < 0) {
                                        collectCycle = 0;
                                    } else if (settings[SETTINGS.SPIKE_2.ordinal()] == 1 && motif != 1 && collectCycle < 1) {
                                        collectCycle = 1;
                                    } else if (settings[SETTINGS.SPIKE_3.ordinal()] == 1 && motif != 2 && collectCycle < 2) {
                                        collectCycle = 2;
                                    } else {
                                        lastCycle = true;
                                    }
                                }
                            } else {
                                motifDone = true;
                                if (allSpikes != 0) {
                                    collectCycle++;
                                } else if (spike1 != 0 && collectCycle < 0) {
                                    collectCycle = 0;
                                } else if (spike2 != 0 && collectCycle < 1) {
                                    collectCycle = 1;
                                } else if (collectCycle < 2) {
                                    collectCycle = 2;
                                }
                            }
                        }
                        if (lastCycle) {
                            state = STATES.DRIVE_OFF_LINE;
                        }
                        break;
                    case LOOK_AT_MOTIF:
                        if (state != previousState) {
                            previousState = state;
                            MM_Position_Data.targetPos.setHeading(150 * alliance);
                        }

                        if (robot.drivetrain.driveDone()) {
                            robot.drivetrain.navigation.visionPortal.setMotif();
                            if (motif != -2) {
                                collectCycle = Math.abs(motif - 2);
                            }
                            state = STATES.COLLECT;
                        }

                        break;
                    case DRIVE_TO_COLLECT:
                        if(collectCycle == settings[MM_OpMode.SETTINGS.SPIKE_2.ordinal()]){
                            spike2Done = true;
                        } else if (chosenSplineList.get(collectCycle) == null){
                            spike1Done = true;
                        }
                        if(settings[SETTINGS.HP_COLLECT.ordinal()] == 0 || settings[SETTINGS.GOAL_SIDE.ordinal()] == 1) {
                            if (state != previousState) {
                                MM_Position_Data.targetPos.setHeading(-90 * alliance);
                                targetHeading = -90;
                                if (collectCycle == 0) {
                                    rotateDone = false;
                                } else {
                                    rotateDone = true;
                                    prepareToSpline(chosenSplineList.get(collectCycle), false);

                                }
                                previousState = state;
                            }

                            if (robot.drivetrain.driveDone() && rotateDone) {
                                previousState = state;
                                if (chosenSplineList.get(collectCycle) == null || (currentSpline != null && splineDone(false))) {
                                    state = STATES.COLLECT;
                                } else if (currentSpline != null) {
                                    setNextSplinePoint(currentSpline, false);

                                }
                            } else if (robot.drivetrain.driveDone() && chosenSplineList.get(collectCycle) == null) {
                                if (settings[SETTINGS.GOAL_SIDE.ordinal()] == 1) {
                                    MM_Position_Data.targetPos.setAll(robot.drivetrain.navigation.firstSpikeX, 33 * alliance, -90 * alliance);
                                } else {
                                    MM_Position_Data.targetPos.setAll(robot.drivetrain.navigation.thirdSpikeX, 33 * alliance, -90 * alliance);
                                }
                                rotateDone = true;
                            }
                            multipleTelemetry.addData("currentTargetX", MM_Position_Data.targetPos.getX());
                            MM_Collector.runCollector = true;
                        } else { //HP collect
                            if(state != previousState){
                                previousState = state;
                                MM_Position_Data.targetPos.setAll(0, 0, 0); //TODO find correct position
                            }

                            if(robot.drivetrain.driveDone()){
                                state = STATES.COLLECT;
                            }
                        }
                        break;
                    case COLLECT:
                        //normal collect
                        if((settings[SETTINGS.HP_COLLECT.ordinal()] == 0) || (settings[SETTINGS.GOAL_SIDE.ordinal()] == 1)) {
                            MM_Position_Data.targetPos.setY(56 * alliance);

                            if (state != previousState) {
                                previousState = state;
                            } else if (robot.drivetrain.driveDone()) {
                                if (!motifDone) {
                                    motifDone = true;
                                    collectCycle = -1;
                                }
                                if ((collectCycle == 1 && (settings[SETTINGS.GOAL_SIDE.ordinal()] < 1) && (settings[SETTINGS.ALL_SPIKES.ordinal()] == 1)) || (collectCycle == 0 && (settings[SETTINGS.ALL_SPIKES.ordinal()] == 1 && settings[SETTINGS.GOAL_SIDE.ordinal()] == 1))) {
                                    state = STATES.OPEN_GATE;
                                } else {
                                    state = STATES.DRIVE_TO_SCORE;
                                }
                            }
                        } else { //HP collect
                            if(robot.launcher.lowerSensorTriggered()){
                                MM_Collector.runCollector = false;
                                state = STATES.DRIVE_TO_SCORE;
                            }
                        }
                        break;
                    case DRIVE_OFF_LINE:
                        if(previousState != state) {
                            previousState = state;
                            if (settings[SETTINGS.GOAL_SIDE.ordinal()] == 1) {
                                MM_Position_Data.targetPos.setAll(0, 24 * alliance, 180);
                            } else {
                                MM_Position_Data.targetPos.setAll(54, 36 * alliance, 180);
                            }
                        } else if (robot.drivetrain.driveDone()) {
                            notDone = false;
                        }
                        break;

                    case OPEN_GATE:
                        if(previousState != state){
                            previousState = state;
                            prepareToSpline(chosenSplineList.get(3), false);
                        } else if (robot.drivetrain.driveDone() && !timerStarted){
                            setNextSplinePoint(currentSpline, false);
                        }

                        if(splineDone(false)){
                            if(timerStarted && gateTime.milliseconds() > 1000){
                                state = STATES.DRIVE_TO_SCORE;
                            }
                            if(!timerStarted){
                                gateTime.reset();
                                timerStarted = true;
                            }
                        }
                        break;
                }

            }
            robot.drivetrain.autoRunDrivetrain();
            robot.collector.autoRunCollector();
            robot.launcher.autoRunLauncher();

            multipleTelemetry.addData("currentCycle", collectCycle);
            multipleTelemetry.addData("dCoeff", MM_Drivetrain.drivePidController.getD_COEFF());
            multipleTelemetry.update();
        }
        MM_Position_Data.targetPos.setAll(0, 0, 0);
    }

    public void setNextSplinePoint(MM_Spline spline, boolean reversed) {
        spline.updateDistanceTraveled(currentSection);
        targetX = spline.getNextPoint(currentSection)[0];
        targetY = spline.getNextPoint(currentSection)[1];
        MM_Position_Data.targetPos.setAll(targetX, targetY * alliance, targetHeading * alliance);
        currentSection = !reversed? currentSection + 1: currentSection - 1;

    }


    public void prepareToSpline(MM_Spline spline, boolean reversed) {
        spline.resetDistanceTraveled();
        currentSpline = spline;
        if(!reversed) {
            currentSection = 0;
        } else {
            currentSection = 20;
        }
        setNextSplinePoint(spline, reversed);
        if(!reversed) {
            currentSection = 0;
        } else {
            currentSection = 20;
        }
        MM_Drivetrain.xErrorThreshold = 4;
        MM_Drivetrain.yErrorThreshold = 4;
        MM_Drivetrain.drivePidController.setD_COEFF(0);
    }


    public boolean splineDone(boolean reversed) {

        if ((currentSection == MM_Autos.SPLINE_DETAIL_LEVEL + 1 && !reversed) || (currentSection == -1 && reversed)) {
            MM_Drivetrain.xErrorThreshold = MM_Drivetrain.X_ERROR_THRESHOLD;
            MM_Drivetrain.yErrorThreshold = MM_Drivetrain.Y_ERROR_THRESHOLD;
            MM_Drivetrain.drivePidController.setD_COEFF(REGULAR_DRIVE_D_CO_EFF);

            currentSection = 0;
            currentSpline = null;
            return true;
        }

        return currentSpline == null;
    }
}

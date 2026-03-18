package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.alliance;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class MM_Launcher {
    private MM_OpMode opMode;

    public static boolean testMode;

    private DcMotorEx launchMotorLeft;
    private DcMotorEx launchMotorRight;
    private Servo pusher;
    private CRServo server;
    private DistanceSensor topRightDistance;
    private DistanceSensor topLeftDistance;
    private DistanceSensor bottomLeftDistance;
    private DistanceSensor bottomRightDistance;
    //public static MM_PID_CONTROLLER serverPID;

    private AnalogInput serverEncoder;
    private AnalogInput pusherEncoder;
    private ElapsedTime launchTime = new ElapsedTime();

    public static final MM_Position projectileTarget = new MM_Position(-62, 62 * alliance, 0); //goal pos

    public static double LAUNCH_ZONE_CO_EFF_AUDIENCE = 2.5; // not really using this
    public static double LAUNCH_ZONE_CO_EFF_CLOSE_AUDIENCE = 2.3;
    public static double LAUNCH_ZONE_CO_EFF_FIELD_CENTER = 2.35;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_MID = 2.55;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_NEAR = 2;

    public String launchZone = "";
    public static double LAUNCH_ZONE_BOUNDARY_CLOSE_AUDIENCE = 130;
    public static double LAUNCH_ZONE_BOUNDARY_FIELD_CENTER = 80;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_MID = 55;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_NEAR = 45;

    private double LAUNCHER_ANGLE = 45;
    public static boolean scoreArtifacts = false;
    public static double PUSHER_BOTTOM_POSITION = .2;
    public static double PUSHER_POSITION_1 = .65;
    public static double PUSHER_POSITION_2 = .77;
    public static double PUSHER_POSITION_3 = .91;
    public static double AXON_ENCODER_CO_EFF = 1;
    private final double PUSHER_POS_THRESHOLD = 0.05;
    private final double PUSHER_POS_BOTTOM_THRESHOLD = 0.33; //this is a bad way to do this but it works

    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height

    private static double SERVER_P_CO_EFF = -.0035;
    private static double SERVER_D_CO_EFF = -0.35;

    public static double serverTuningPCoEff = SERVER_P_CO_EFF;
    public static double serverTuningDCoEff = SERVER_D_CO_EFF;
    public static boolean tuningServerCoEffs;

    private static MM_PID_CONTROLLER serverPIDController = new MM_PID_CONTROLLER(SERVER_P_CO_EFF, 0, SERVER_D_CO_EFF);

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    public boolean artifactAtTop = true;
    private boolean serverIsReady = false;
    private boolean launching = false;
    private boolean attemptedShot;
    public static int serverStopPoint = 110;
    double serverNormalizedError = 0;
    double launchAttempts;

    public MM_Launcher(MM_OpMode opMode) {
        this.opMode = opMode;

        launchMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorLeft");
        launchMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorRight");
        launchMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        pusher = opMode.hardwareMap.get(Servo.class, "pusher");
        pusherEncoder = opMode.hardwareMap.get(AnalogInput.class, "pusherEncoder");
        pusher.setPosition(PUSHER_BOTTOM_POSITION);
        server = opMode.hardwareMap.get(CRServo.class, "server");
        serverEncoder = opMode.hardwareMap.get(AnalogInput.class, "serverEncoder");
        //setServerForInit(); TODO fix method to make it stop server at right place in init
        topLeftDistance = opMode.hardwareMap.get(DistanceSensor.class, "topLeftDistance");
        topRightDistance = opMode.hardwareMap.get(DistanceSensor.class, "topRightDistance");
        bottomLeftDistance = opMode.hardwareMap.get(DistanceSensor.class, "bottomLeftDistance");
        bottomRightDistance = opMode.hardwareMap.get(DistanceSensor.class, "bottomRightDistance");
        //serverPID = new MM_PID_CONTROLLER(SERVER_P_CO_EFF, 0, 0);

        launchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLauncher() {
        double pusherPos = getAxonPosition(pusherEncoder);
        double serverPos = getAxonPosition(serverEncoder);

        calculateAndSetTargetLauncherVelocity();

        if (opMode.gamepad2.yWasPressed()) { //bring pusher to bottom
            pusher.setPosition(PUSHER_BOTTOM_POSITION);
            opMode.multipleTelemetry.addData("Pusher Position", "Bottom Position");
        } else if (opMode.gamepad2.bWasPressed()) { //TODO add velocity up to speed
            if (pusherPosWithinThreshold(PUSHER_BOTTOM_POSITION)) {
                pusher.setPosition(PUSHER_POSITION_1);
                opMode.multipleTelemetry.addData("Pusher Position", "Position 1");
            } else if (pusherPosWithinThreshold(PUSHER_POSITION_1)) {
                pusher.setPosition(PUSHER_POSITION_2);
                opMode.multipleTelemetry.addData("Pusher Position", "Position 2");
            } else if (pusherPosWithinThreshold(PUSHER_POSITION_2)) {
                pusher.setPosition(PUSHER_POSITION_3);
                opMode.multipleTelemetry.addData("Pusher Position", "Position 3");
            } else if (pusherPosWithinThreshold(PUSHER_POSITION_3)) {
                pusher.setPosition(PUSHER_BOTTOM_POSITION);
                opMode.multipleTelemetry.addData("Pusher Position", "Bottom Position");
            }
        }

//        double pusherPos = getAxonDegrees(pusherEncoder);
//        double serverPos = getAxonDegrees(serverEncoder);
//        double serverError = serverPos - serverStopPoint;
//
//        setTargetLauncherVelocity();
//        haveArtifactAtTop();
//
//        boolean velocityCorrect = Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50;
//
//        if (!launching && currentGamepad2.left_trigger > 0) { //rapid fire
//            launching = true;
//            serverStopPoint = 280;
//        }
//        if (serverStopPoint == 280 && Math.abs(serverError) < 150) { //server out of the way
//            if (velocityCorrect) {
//                pusher.setPosition(PUSHER_POSITION_1);
//            }
//            serverStopPoint = 281;
//        } else if (Math.abs(pusherPos / 360 - PUSHER_POSITION_3) <= .02) { //done launching
//            serverStopPoint = 45; //used to be 110 but this hopefully stops us from colleting 4 artifacts
//            pusher.setPosition(PUSHER_BOTTOM_POSITION);
//            opMode.robot.drivetrain.autoLockInToAprilTag = true;
//        }
//
//        //handle semi-rapid-fire
//        if (launching && serverStopPoint == 281) {
//            if (pusher.getPosition() >= PUSHER_POSITION_2 - .01) {
//                if (velocityCorrect) {
//                    pusher.setPosition(PUSHER_POSITION_3);
//                }
//            } else if (pusher.getPosition() >= PUSHER_POSITION_1 - .01) {
//                if (velocityCorrect) {
//                    pusher.setPosition(PUSHER_POSITION_2);
//                }
//            } else {
//                if (velocityCorrect) {
//                    pusher.setPosition(PUSHER_POSITION_1);
//                }
//            }
//        }
//
//        if (tuningServerCoEffs) {
//            serverPIDController.setP_COEFF(serverTuningPCoEff);
//            serverPIDController.setD_COEFF(serverTuningDCoEff);
//
//            server.setPower(serverPIDController.getPID(serverNormalizedError));
//            opMode.multipleTelemetry.addData("serverError", serverError);
//            opMode.multipleTelemetry.addData("serverP", serverPIDController.getP());
//            opMode.multipleTelemetry.addData("serverD", serverPIDController.getD());
//        }
//
//        opMode.multipleTelemetry.addData("serverTarget", serverStopPoint);
//        opMode.multipleTelemetry.addData("launcherTargetSpeed", targetLauncherVelocity);
//        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
//        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());
//        opMode.multipleTelemetry.addData("servoEncoder", serverPos);
//        opMode.multipleTelemetry.addData("serverSpeed", server.getPower());
////        opMode.multipleTelemetry.addData("dist Left", topLeftDistance.getDistance(DistanceUnit.MM));
////        opMode.multipleTelemetry.addData("dist Right", topRightDistance.getDistance(DistanceUnit.MM));
////        opMode.multipleTelemetry.addData("dist Lower Left", bottomLeftDistance.getDistance(DistanceUnit.MM));
////        opMode.multipleTelemetry.addData("dist Lower Right", bottomRightDistance.getDistance(DistanceUnit.MM));
//
//        //if (launching){
//        if (!artifactAtTop && currentGamepad2.b && !previousGamepad2.b) {
//            if (pusher.getPosition() >= PUSHER_POSITION_3 - .01) {
//                pusher.setPosition(PUSHER_BOTTOM_POSITION);
//            } else if (pusher.getPosition() >= PUSHER_POSITION_2 - .01) {
//                pusher.setPosition(PUSHER_POSITION_3);
//            } else if (pusher.getPosition() >= PUSHER_POSITION_1 - .01) {
//                pusher.setPosition(PUSHER_POSITION_2);
//            } else {
//                pusher.setPosition(PUSHER_POSITION_1);
//            }
//        }
//        if (currentGamepad2.y) {
//            pusher.setPosition(PUSHER_BOTTOM_POSITION);
//        }
//        opMode.multipleTelemetry.addData("servo pos", pusher.getPosition());
//        // }
//
//        if (haveArtifactAtTop() || launching) { //TODO only set velocity once
//            launchMotorLeft.setVelocity(targetLauncherVelocity);
//            launchMotorRight.setVelocity(targetLauncherVelocity);
//        } else {
//            launchMotorLeft.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
//            launchMotorRight.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
//        }
//
//
//        if (opMode.gamepad2.dpad_left) {
//            server.setPower(-0.2);
//        } else if (opMode.gamepad2.dpad_right) {
//            server.setPower(0.2);
//        } else {
//            if (Math.abs(server.getPower()) > 0 && !launching) {
//                //server.setPower(0);
//            }
//            if ((pusher.getPosition() >= PUSHER_POSITION_1 - .1 && !launching && artifactAtTop && currentGamepad2.right_trigger > 0 && Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50) || (testMode && currentGamepad2.a && !previousGamepad2.a)) {
////            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION); TODO fix the lower feed arm
//                launching = true;
//            }
//            opMode.multipleTelemetry.addData("launching", launching);
//            opMode.multipleTelemetry.addData("artifact at top", artifactAtTop);
//            opMode.multipleTelemetry.addData("launchError", launchMotorLeft.getVelocity() - targetLauncherVelocity);
//            opMode.multipleTelemetry.addData("controller right trigger", currentGamepad2.right_trigger);
//
//            //opMode.multipleTelemetry.addData("colors", "red %d, green %d, blue %d", peephole.red(), peephole.green(), peephole.blue());
//
//            if (launching) {
//                serverNormalizedError = calculateNormalizedServerError(serverStopPoint, true);
//                if (Math.abs(serverNormalizedError) < 90) {
//                    launching = false;
//                }
//            } else {
//                serverNormalizedError = calculateNormalizedServerError(serverStopPoint, false);
//            }
//            server.setPower(serverPIDController.getPID(serverNormalizedError));
//            opMode.multipleTelemetry.addData("serverNormalizedError", serverNormalizedError);
//        }
    }

    public void autoRunLauncher() {
        double pusherPos = getAxonDegrees(pusherEncoder);
        double serverPos = getAxonDegrees(serverEncoder);
        double serverError = serverPos - serverStopPoint;
        double targetLauncherVelocity = calculateAndSetTargetLauncherVelocity(); //also sets launch motor velocity

        if (scoreArtifacts && opMode.robot.drivetrain.driveDone()) {

            if (!attemptedShot) {
                if (serverStopPoint < 300) { //rapid fire
                    serverStopPoint = 300;
                } else if (serverStopPoint == 300 && Math.abs(serverError) < 150) {
                    pusher.setPosition(PUSHER_POSITION_3);
                    serverStopPoint = 301;
                } else if (Math.abs(pusherPos / 360 - PUSHER_POSITION_3) < .017) {
                    serverStopPoint = 60;
                    attemptedShot = true;
                    pusher.setPosition(PUSHER_BOTTOM_POSITION);
                }
            }

            if (attemptedShot && pusherPos / 360 < .53 + .02) {
                attemptedShot = false;
                //launchAttempts++;

                // if(launchAttempts > 1) {
                scoreArtifacts = false;
                // launchAttempts = 0;
                //} else {
                // launchAttempts++;

                //}

            }

//            if (!launching) {
//                if (pusher.getPosition() >= PUSHER_POSITION_3 - .02) {
//                    pusher.setPosition(PUSHER_BOTTOM_POSITION);
//                    scoreArtifacts = false;
//                } else {
//                    if (pusher.getPosition() <= PUSHER_BOTTOM_POSITION + .01) {
//                        pusher.setPosition(PUSHER_POSITION_1);
//                    } else if (pusher.getPosition() >= PUSHER_POSITION_2 - .03) {
//                        pusher.setPosition(PUSHER_POSITION_3);
//                    } else if (pusher.getPosition() >= PUSHER_POSITION_1 - .01) {
//                        pusher.setPosition(PUSHER_POSITION_2);
//                    }
//                    launching = true;
//                }
//            }
            opMode.multipleTelemetry.addData("launching", launching);
            opMode.multipleTelemetry.addData("serverisready", serverIsReady);
            opMode.multipleTelemetry.addData("servo pos", pusherPos / 360);

            if (Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50 && launching) {
                if (pusherPos / 360 >= pusher.getPosition() - .02 && pusher.getPosition() >= PUSHER_BOTTOM_POSITION - .02) {
                    server.setPower(Math.abs(serverPIDController.getPID(serverError < serverStopPoint ? serverError : serverStopPoint + (360 - serverPos))));
                    if (serverPos < 90) {
                        serverIsReady = false;
                    }

                }
            } else {
                server.setPower(serverPIDController.getPID(serverError));
            }
//            if (!serverIsReady && server.getPower() < .7 && launching) {
//                serverIsReady = true;
//                launching = false;
//            }

        } else {
            server.setPower(serverPIDController.getPID(serverError));
        }
    }

    private double calculateAndSetTargetLauncherVelocity() {
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())); // unit is inches
        double metersPerSecond = Math.sqrt((9.81 * launchDistance * 0.0254) / Math.sin(LAUNCHER_ANGLE * 2));
        double ticksPerSecond = Math.abs(metersPerSecond * TICKS_PER_METER);
        double calculatedVelocity;

        if (launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_NEAR) {
            calculatedVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_NEAR;
            launchZone = "Goal Near";
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_MID) {
            calculatedVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_MID;
            launchZone = "Goal Mid";
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_FIELD_CENTER) {
            calculatedVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_FIELD_CENTER;
            launchZone = "Field Center";
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_CLOSE_AUDIENCE) {
            calculatedVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_CLOSE_AUDIENCE;
            launchZone = "Close Audience";
        } else {
            calculatedVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_AUDIENCE;
            launchZone = "Audience";
        }

        launchMotorLeft.setVelocity(calculatedVelocity);
        launchMotorRight.setVelocity(calculatedVelocity);

        opMode.multipleTelemetry.addData("launchDistance (inches)", launchDistance);
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
        opMode.multipleTelemetry.addData("Launch Zone", launchZone);

        return calculatedVelocity;
    }

    private boolean pusherPosWithinThreshold(double pusherPosTarget){
        double pusherPos = getAxonPosition(pusherEncoder);
        double error = pusherPosTarget - pusherPos;

        opMode.multipleTelemetry.addData("pusher position", pusherPos);
        opMode.multipleTelemetry.addData("error", error);

        if (pusher.getPosition() == PUSHER_BOTTOM_POSITION) {
            return error < PUSHER_POS_BOTTOM_THRESHOLD;
        }

        return Math.abs(error) < PUSHER_POS_THRESHOLD;
    }

    private double calculateNormalizedServerError(double target, boolean launching) {
        double currentPos = getAxonDegrees(serverEncoder);
        opMode.multipleTelemetry.addData("servoEncoder", currentPos);
        double error;
        if (launching) {
            error = target <= (currentPos + 15) ? -(target + (360 - currentPos)) : -(target - currentPos);
        } else {
            error = (Math.abs(target - currentPos) < Math.abs((target - 360) - currentPos)) ? currentPos - target : -((target - 360) - currentPos);
        }
        opMode.multipleTelemetry.addData("serverNormalizedError", error);
        return error;
    }

    private boolean haveArtifactAtTop() {
        artifactAtTop = topLeftDistance.getDistance(DistanceUnit.MM) < 20 || topRightDistance.getDistance(DistanceUnit.MM) < 33;
        return true;
    }

    public boolean lowerSensorTriggered() {
        return bottomLeftDistance.getDistance(DistanceUnit.MM) < 40 || bottomRightDistance.getDistance(DistanceUnit.MM) < 40;
    }

    public boolean lowerFeedArmReady() {
        opMode.multipleTelemetry.addData("encoder pos", getAxonDegrees(pusherEncoder));
        return getAxonDegrees(pusherEncoder) < 200;
    }

    private double getAxonDegrees(AnalogInput encoder) {
        return ((encoder.getVoltage() / 3.3) * 360);
    }

    private double getAxonPosition(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3;
    }

    public void setServerForInit() {
        double serverError = Math.abs(getAxonDegrees(serverEncoder) - serverStopPoint);

        if (!serverIsReady) {
            if (getAxonDegrees(serverEncoder) < serverStopPoint) {
                server.setPower(serverError * SERVER_P_CO_EFF);
            } else {
                serverIsReady = true;
                server.setPower(0);
            }
        }
    }

    public void updateProjectileTarget() {
        projectileTarget.setY(65 * alliance);
    }
}


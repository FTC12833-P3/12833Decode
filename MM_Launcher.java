package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.alliance;
import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad2;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad2;

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

    public static final MM_Position projectileTarget = new MM_Position(-65, 65 * alliance, 0); //goal pos

    public static double LAUNCH_ZONE_CO_EFF_AUDIENCE = 2.4;
    public static double LAUNCH_ZONE_CO_EFF_FIELD_CENTER = 2.35;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_MID = 2.7;
    public static double LAUNCH_ZONE_CO_EFF_GOAL_NEAR = 0;

    public static double LAUNCH_ZONE_BOUNDARY_FIELD_CENTER = 100;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_MID = 55;
    public static double LAUNCH_ZONE_BOUNDARY_GOAL_NEAR = 45;

    private double LAUNCHER_ANGLE = 45;
    public static boolean scoreArtifacts = false;
    public static double targetLauncherVelocity = 10000;
    public static double PUSHER_BOTTOM_POSITION = .2;
    public static double PUSHER_POSITION_1 = .65;
    public static double PUSHER_POSITION_2 = .77;
    public static double PUSHER_POSITION_3 = .885;
    public static double AXON_ENCODER_CO_EFF = 1;

    private final double FINAL_PROJECTILE_HEIGHT = 26.5; //height above launch height


    private static double SERVER_P_CO_EFF = -.0035;
    private static double SERVER_D_CO_EFF = -0.35;

    public static double serverTuningPCoEff = SERVER_P_CO_EFF;
    public static double serverTuningDCoEff = SERVER_D_CO_EFF;
    public static boolean tuningServerCoEffs;

    private static MM_PID_CONTROLLER serverPIDController = new MM_PID_CONTROLLER(SERVER_P_CO_EFF, 0 , SERVER_D_CO_EFF);

    private final double TICKS_PER_REV = 28;
    private final double WHEEL_DIAMETER = 77.75; //mm 75.75 for ordered wheels, 70.95 for custom
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double TICKS_PER_METER = (TICKS_PER_REV / CIRCUMFERENCE) * 1000;

    private final double SLOW_SPEED_CO_EFF = .25;

    public boolean artifactAtTop = true;
    private boolean serverIsReady = false;
    private boolean launching = false;
    private boolean attemptedShot;
    public static int serverStopPoint = 120;

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
        setTargetLauncherVelocity();
        haveArtifactAtTop();

        if(currentGamepad2.left_trigger > 0){ //rapid fire
            serverStopPoint = 300;
        }
        if(serverStopPoint == 300 && Math.abs(getAxonDegrees(serverEncoder) - serverStopPoint) < 150){
            pusher.setPosition(PUSHER_POSITION_3);
            serverStopPoint = 301;
        } else if(Math.abs(getAxonDegrees(pusherEncoder) / 360 - PUSHER_POSITION_3) < .01){
            serverStopPoint = 120;
        }

        double serverError = getAxonDegrees(serverEncoder) - serverStopPoint;
        if(tuningServerCoEffs) {
            serverPIDController.setP_COEFF(serverTuningPCoEff);
            serverPIDController.setD_COEFF(serverTuningDCoEff);

            server.setPower(serverPIDController.getPID(serverError));
            opMode.multipleTelemetry.addData("serverError", serverError);
            opMode.multipleTelemetry.addData("serverP", serverPIDController.getP());
            opMode.multipleTelemetry.addData("serverD", serverPIDController.getD());
        }

        opMode.multipleTelemetry.addData("serverTarget", serverStopPoint);

        opMode.multipleTelemetry.addData("launcherTargetSpeed", targetLauncherVelocity);
        opMode.multipleTelemetry.addData("launcherSpeedL", launchMotorLeft.getVelocity());
        opMode.multipleTelemetry.addData("launcherSpeedR", launchMotorRight.getVelocity());
        opMode.multipleTelemetry.addData("servoEncoder", getAxonDegrees(serverEncoder));
        opMode.multipleTelemetry.addData("serverSpeed", server.getPower());
        opMode.multipleTelemetry.addData("dist Left", topLeftDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Right", topRightDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Lower Left", bottomLeftDistance.getDistance(DistanceUnit.MM));
        opMode.multipleTelemetry.addData("dist Lower Right", bottomRightDistance.getDistance(DistanceUnit.MM));

        //if (launching){
        if (!artifactAtTop && currentGamepad2.b && !previousGamepad2.b) {
            if (pusher.getPosition() >= PUSHER_POSITION_3 -.01) {
                pusher.setPosition(PUSHER_BOTTOM_POSITION);
            } else if (pusher.getPosition() >= PUSHER_POSITION_2 -.01) {
                pusher.setPosition(PUSHER_POSITION_3);
            } else if (pusher.getPosition() >= PUSHER_POSITION_1 -.01) {
                pusher.setPosition(PUSHER_POSITION_2);
            } else {
                pusher.setPosition(PUSHER_POSITION_1);
            }
        }
        if (currentGamepad2.y) {
            pusher.setPosition(PUSHER_BOTTOM_POSITION);
        }
        opMode.multipleTelemetry.addData("servo pos", pusher.getPosition());
        // }

        if (haveArtifactAtTop() || launching) { //TODO only set velocity once
            launchMotorLeft.setVelocity(targetLauncherVelocity);
            launchMotorRight.setVelocity(targetLauncherVelocity);
        } else {
            launchMotorLeft.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
            launchMotorRight.setVelocity(targetLauncherVelocity * SLOW_SPEED_CO_EFF);
        }


        if (opMode.gamepad2.dpad_left) {
            server.setPower(-0.2);
        } else if (opMode.gamepad2.dpad_right) {
            server.setPower(0.2);
        } else {
            if(Math.abs(server.getPower()) > 0 && !launching) {
                //server.setPower(0);
            }
            if ((pusher.getPosition() >= PUSHER_POSITION_1 - .1 && !launching && artifactAtTop && currentGamepad2.right_trigger > 0 && Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50) || (testMode && currentGamepad2.a && !previousGamepad2.a)) {
//            lowerFeedArm.setPosition(LOWER_FEED_BAR_TOP_POSITION); TODO fix the lower feed arm
                server.setPower(1);
                launching = true;
            }
            opMode.multipleTelemetry.addData("launching", launching);
            opMode.multipleTelemetry.addData("artifact at top",artifactAtTop);
            opMode.multipleTelemetry.addData("launchError", launchMotorLeft.getVelocity() - targetLauncherVelocity);
            opMode.multipleTelemetry.addData("controller right trigger", currentGamepad2.right_trigger);

            //opMode.multipleTelemetry.addData("colors", "red %d, green %d, blue %d", peephole.red(), peephole.green(), peephole.blue());

            if (launching) {
                server.setPower(Math.abs(serverPIDController.getPID(getAxonDegrees(serverEncoder) < serverStopPoint ? serverError : serverStopPoint + (360 - getAxonDegrees(serverEncoder)))));
                if(getAxonDegrees(serverEncoder) < 100) {
                    serverIsReady = false;
                } else if (!serverIsReady && getAxonDegrees(serverEncoder) > 180) {
                    serverIsReady = true;
                    launching = false;
                }
            } else {
                server.setPower(serverPIDController.getPID(serverError));
            }

        }
    }

    public void autoRunLauncher() {
        setTargetLauncherVelocity();
        launchMotorLeft.setVelocity(targetLauncherVelocity);
        launchMotorRight.setVelocity(targetLauncherVelocity);
        double serverError = getAxonDegrees(serverEncoder) - serverStopPoint;

        if (scoreArtifacts && opMode.robot.drivetrain.driveDone()) {

            if(!attemptedShot) {
                if (serverStopPoint < 300) { //rapid fire
                    serverStopPoint = 300;
                } else if (serverStopPoint == 300 && Math.abs(getAxonDegrees(serverEncoder) - serverStopPoint) < 150) {
                    pusher.setPosition(PUSHER_POSITION_3);
                    serverStopPoint = 301;
                } else if (Math.abs(getAxonDegrees(pusherEncoder) / 360 - PUSHER_POSITION_3) < .01) {
                    serverStopPoint = 120;
                    attemptedShot = true;
                    pusher.setPosition(PUSHER_BOTTOM_POSITION);
                }
            }

            if(attemptedShot && getAxonDegrees(pusherEncoder) / 360 < .53 + .01){
                attemptedShot = false;
                scoreArtifacts = false;
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
            opMode.multipleTelemetry.addData("servoEncoder", getAxonDegrees(serverEncoder));
            opMode.multipleTelemetry.addData("serverisready", serverIsReady);
            opMode.multipleTelemetry.addData("servo pos", getAxonDegrees(pusherEncoder) / 360);




            if (Math.abs(launchMotorLeft.getVelocity() - targetLauncherVelocity) < 50 && launching) {
                if (getAxonDegrees(pusherEncoder) / 360 >= pusher.getPosition() - .02 && pusher.getPosition() >= PUSHER_BOTTOM_POSITION -.02) {
                    server.setPower(Math.abs(serverPIDController.getPID(getAxonDegrees(serverEncoder) < serverStopPoint ? serverError : serverStopPoint + (360 - getAxonDegrees(serverEncoder)))));
                    if(getAxonDegrees(serverEncoder) < 90) {
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

    private void setTargetLauncherVelocity() {
        double launchDistance = Math.abs(Math.hypot(projectileTarget.getX() - opMode.robot.drivetrain.navigation.getX(),
                projectileTarget.getY() - opMode.robot.drivetrain.navigation.getY())); // unit is inches
        double metersPerSecond = Math.sqrt((9.81 * launchDistance * 0.0254) / Math.sin(LAUNCHER_ANGLE * 2));
        double ticksPerSecond = Math.abs(metersPerSecond * TICKS_PER_METER);

        if (launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_NEAR) {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_NEAR;
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_GOAL_MID) {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_GOAL_MID;
        } else if (launchDistance <= LAUNCH_ZONE_BOUNDARY_FIELD_CENTER) {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_FIELD_CENTER;
        } else {
            targetLauncherVelocity = ticksPerSecond * LAUNCH_ZONE_CO_EFF_AUDIENCE;
        }

        opMode.multipleTelemetry.addData("launchDistance (inches)", launchDistance);
        opMode.multipleTelemetry.addData("metersPerSecond", metersPerSecond);
    }

    private boolean haveArtifactAtTop() {
        artifactAtTop = topLeftDistance.getDistance(DistanceUnit.MM) < 20 || topRightDistance.getDistance(DistanceUnit.MM) < 33;
        return true;
    }

    public boolean lowerSensorTriggered() {
        return bottomLeftDistance.getDistance(DistanceUnit.MM) < 35 || bottomRightDistance.getDistance(DistanceUnit.MM) < 35;
    }

    public boolean lowerFeedArmReady() {
        opMode.multipleTelemetry.addData("encoder pos", getAxonDegrees(pusherEncoder));
        return getAxonDegrees(pusherEncoder) < 200;
    }

    private double getAxonDegrees(AnalogInput encoder) {
        return ((encoder.getVoltage() / 3.3) * 360);
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


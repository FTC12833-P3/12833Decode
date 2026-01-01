package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class MM_Drivetrain {
    MM_OpMode opMode;
    MM_Position_Data navigation;
    public static MM_PID_CONTROLLER DrivePidController = new MM_PID_CONTROLLER(0.2, 0, 30); //TODO find correct PID coefficients

    private final DcMotorEx flMotor;
    private final DcMotorEx frMotor;
    private final DcMotorEx blMotor;
    private final DcMotorEx brMotor;

    public static boolean tuningDrivePID = false;
    public static double tuningDrivePCoEff = .2;
    public static double tuningDriveICoEff = 0;
    public static double tuningDriveDCoEff = 30;


    private static final double SLOW_MODE_POWER = .5;

    public static double xErrorThreshold = 1.5; //TODO fix threshold values (used to be .5)
    public static double yErrorThreshold = 1.5;
    public static double headingErrorThreshold = 3;
    public static double rotatePCoEff = .05;

    public static final double ROTATE_P_CO_EFF = .05;

    public static final double X_ERROR_THRESHOLD = 1.5;
    public static final double Y_ERROR_THRESHOLD = 1.5;

    public static double desiredPower = 1;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    private boolean slowMode = false;
    private boolean positonLocked = false;
    private boolean rotateLocked = false;


    private double pidError;
    private double xError = 0;
    private double yError = 0;
    private double headingError = 0;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        navigation = new MM_Position_Data(opMode);

        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = -opMode.gamepad1.right_stick_x;

        if(currentGamepad1.y && !previousGamepad1.y){ //toggle lock position
            navigation.updatePosition();
            positonLocked = !positonLocked;
            MM_Position_Data.targetPos.setAll(navigation.getX(), navigation.getY(), navigation.getHeading());
        }

        if(currentGamepad1.x && !previousGamepad1.x){
            rotateLocked = !rotateLocked;
        }

        if(rotateLocked){
            navigation.updatePosition();
            MM_Position_Data.targetPos.setHeading(calculateDesiredAngle());
            opMode.multipleTelemetry.addData("targetAngle", MM_Position_Data.targetPos.getHeading());
            headingError = getNormalizedHeadingError();
            rotatePower = headingError * ROTATE_P_CO_EFF;
        }

        if(positonLocked){
            navigation.updatePosition();
            xError = MM_Position_Data.targetPos.getX() - navigation.getX();
            yError = MM_Position_Data.targetPos.getY() - navigation.getY();
            if(!rotateLocked) {
                headingError = getNormalizedHeadingError();
            }

            if(tuningDrivePID){
                DrivePidController.setP_COEFF(tuningDrivePCoEff);
                DrivePidController.setD_COEFF(tuningDriveDCoEff);
            }

            double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
            double theta = moveAngle - navigation.getHeading() + 45;

            double PID = DrivePidController.getPID(Math.hypot(xError, yError));

            flPower = (2 * Math.cos(Math.toRadians(theta)) * PID) - rotatePower;
            frPower = (2 * Math.sin(Math.toRadians(theta)) * PID) + rotatePower;
            blPower = (2 * Math.sin(Math.toRadians(theta)) * PID) - rotatePower; //I double checked these lines.
            brPower = (2 * Math.cos(Math.toRadians(theta)) * PID) + rotatePower;
        }

        if (currentGamepad1.a && !previousGamepad1.a && !currentGamepad1.start) {
            slowMode = !slowMode;
        }
        if (!positonLocked) {
            flPower = drivePower + strafePower - rotatePower;
            frPower = drivePower - strafePower + rotatePower;
            blPower = drivePower - strafePower - rotatePower;
            brPower = drivePower + strafePower + rotatePower;
        }
        setDrivePowers();
    }

    private void normalize() {
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1) {
            flPower = (flPower / maxPower) * desiredPower;
            frPower = (frPower / maxPower) * desiredPower;
            blPower = (blPower / maxPower) * desiredPower;
            brPower = (brPower / maxPower) * desiredPower;
        }
    }

    private void setDrivePowers() {
        normalize();

        opMode.multipleTelemetry.addData("flPower", flPower);
        opMode.multipleTelemetry.addData("frPower", frPower);
        opMode.multipleTelemetry.addData("blPower", blPower);
        opMode.multipleTelemetry.addData("brPower", brPower);

        if (slowMode) {
            flPower *= SLOW_MODE_POWER;
            frPower *= SLOW_MODE_POWER;
            blPower *= SLOW_MODE_POWER;
            brPower *= SLOW_MODE_POWER;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    private void setDrivePowersToZero() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
    public void enableBrakes(){
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoRunDrivetrain() {
        navigation.updatePosition();
        xError = MM_Position_Data.targetPos.getX() - navigation.getX();
        yError = MM_Position_Data.targetPos.getY() - navigation.getY();
        headingError = getNormalizedHeadingError();

        if(tuningDrivePID){
            DrivePidController.setP_COEFF(tuningDrivePCoEff);
            DrivePidController.setD_COEFF(tuningDriveDCoEff);
        }

        double rotateVector = headingError * rotatePCoEff;
        double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
        double theta = moveAngle - navigation.getHeading() + 45;

        double PID = DrivePidController.getPID(Math.hypot(xError, yError));
        opMode.multipleTelemetry.addData("PIDpower", PID);

        flPower = (2 * Math.cos(Math.toRadians(theta)) * PID) - rotateVector;
        frPower = (2 * Math.sin(Math.toRadians(theta)) * PID) + rotateVector;
        blPower = (2 * Math.sin(Math.toRadians(theta)) * PID) - rotateVector; //I double checked these lines.
        brPower = (2 * Math.cos(Math.toRadians(theta)) * PID) + rotateVector;

        setDrivePowers();
        opMode.multipleTelemetry.addData("zMove angle", moveAngle);
        opMode.multipleTelemetry.addData("zHeading error", headingError);
        opMode.multipleTelemetry.addData("zXError", xError);
        opMode.multipleTelemetry.addData("zYError", yError);
        opMode.multipleTelemetry.addData("zTheta", theta);
        opMode.multipleTelemetry.addData("D", DrivePidController.getD());
        opMode.multipleTelemetry.addData("rate of change of hypot error", DrivePidController.getD() / DrivePidController.getD_COEFF());
        opMode.multipleTelemetry.addData("P", DrivePidController.getP());
        opMode.multipleTelemetry.addData("hypot error", Math.hypot(xError, yError));
    }

    public boolean driveDone() {
        return Math.abs(xError) < xErrorThreshold && Math.abs(yError) < yErrorThreshold && Math.abs(headingError) < headingErrorThreshold;
    }

    private double getNormalizedHeadingError() {
        double error = MM_Position_Data.targetPos.getHeading() - navigation.getHeading();

        error = (error >= 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }
    private double calculateDesiredAngle(){
        double xError = MM_Launcher.projectileTarget.getX() - navigation.getX();
        double yError = MM_Launcher.projectileTarget.getY() - navigation.getY();
        double angle = Math.toDegrees(Math.atan2(xError, yError)) + 180;
        opMode.multipleTelemetry.addData("desiredAngle", angle);
        opMode.multipleTelemetry.addData("launchXError", xError);
        opMode.multipleTelemetry.addData("launchYError", yError);

        return angle;
    }
}

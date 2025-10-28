package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class MM_Drivetrain {
    MM_OpMode opMode;
    MM_Position_Data navigation;
    public static MM_PID_CONTROLLER pidController = new MM_PID_CONTROLLER(0.2, 0, 30); //TODO find correct PID coefficients

    private final DcMotorEx flMotor;
    private final DcMotorEx frMotor;
    private final DcMotorEx blMotor;
    private final DcMotorEx brMotor;

    private static final double SLOW_MODE_POWER = .5;
    private final double HEADING_ERROR_THRESHOLD = 3;

    public static double ROTATE_P_CO_EFF = .05;
    public static double X_ERROR_THRESHOLD = 1.5; //TODO fix threshold values (used to be .5)
    public static double Y_ERROR_THRESHOLD = 1.5;

    public static double desiredPower = 1;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    private boolean slowMode = false;

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

        if (currentGamepad1.a && !previousGamepad1.a && !currentGamepad1.start) {
            slowMode = !slowMode;
        }

        flPower = drivePower + strafePower - rotatePower;
        frPower = drivePower - strafePower + rotatePower;
        blPower = drivePower - strafePower - rotatePower;
        brPower = drivePower + strafePower + rotatePower;
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

        double rotateVector = headingError * ROTATE_P_CO_EFF;
        double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
        double theta = moveAngle - navigation.getHeading() + 45;

        double PID = pidController.getPID(Math.hypot(xError, yError));
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
        opMode.multipleTelemetry.addData("D", pidController.getD());
        opMode.multipleTelemetry.addData("rate of change of hypot error", pidController.getD() / MM_PID_CONTROLLER.D_COEFF);
        opMode.multipleTelemetry.addData("P", pidController.getP());
        opMode.multipleTelemetry.addData("hypot error", Math.hypot(xError, yError));
    }

    public boolean driveDone() {
        return Math.abs(xError) < X_ERROR_THRESHOLD && Math.abs(yError) < Y_ERROR_THRESHOLD && Math.abs(headingError) < HEADING_ERROR_THRESHOLD;
    }

    private double getNormalizedHeadingError() {
        double error = MM_Position_Data.targetPos.getHeading() - navigation.getHeading();

        error = (error >= 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }
}

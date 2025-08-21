package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class MM_VisionPortal {
    public MM_OpMode opMode;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private Limelight3A limelight;

    public static double intrinsicX;
    public static double previousIntrinsicX;

    public static int startingTag;

    public MM_VisionPortal(MM_OpMode opMode){
        this.opMode = opMode;
        init();
    }

    public Pose2D setPosFromApriltag(){
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if(!detections.isEmpty() && detections.get(0).ftcPose != null){
            opMode.multipleTelemetry.addData("xIntrins", round2Dec(detections.get(0).ftcPose.x));
            opMode.multipleTelemetry.addData("yIntrins", round2Dec(detections.get(0).ftcPose.y));
            opMode.multipleTelemetry.addData("yawIntrins", round2Dec(detections.get(0).ftcPose.yaw));
            startingTag = detections.get(0).id;
            previousIntrinsicX = intrinsicX;
            intrinsicX = detections.get(0).ftcPose.x;
            return new Pose2D(DistanceUnit.INCH, detections.get(0).robotPose.getPosition().x,
                    detections.get(0).robotPose.getPosition().y, AngleUnit.DEGREES, detections.get(0).robotPose.getOrientation().getYaw());
        }

        return null;
    }



    private void init(){
        final CameraStreamProcessor cameraStreamProcessor = new CameraStreamProcessor();

        aprilTagProcessor =  new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setCameraPose(new Position(DistanceUnit.INCH, -.625, 6.8125, 0.0, 0L),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 38, 0L))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(cameraStreamProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);


        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1); //switch to the yellow sample detector pipeline
        limelight.start();
    }

    private double round2Dec(double inDouble) {
        return Math.round(inDouble * 100) / 100.0;
    }

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}

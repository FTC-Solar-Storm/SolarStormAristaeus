package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorVision {

    private OpenCvWebcam webcam;
    private final YellowPipeline pipeline;

    private boolean autoPickup = false;
    private volatile boolean cameraOpened = false;

    // CAMERA SETTINGS
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;
    private static final int CAMERA_FPS = 30;

    // AUTO PICKUP SETTINGS
    private static final double TURN_MULTIPLIER = 0.025;
    private static final double MAX_TURN_POWER = 0.5;
    private static final double FORWARD_POWER = 0.5;
    private static final double ALIGNMENT_THRESHOLD = 5.0;

    // Assumed horizontal field of view.
    // Calibrate this value for your camera if needed.
    private static final double HORIZONTAL_FOV = 69.0;

    public ColorVision(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        WebcamName cameraName = hardwareMap.get(
                WebcamName.class,
                "Webcam 1"
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                cameraName,
                cameraMonitorViewId
        );

        pipeline = new YellowPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {

                    @Override
                    public void onOpened() {

                        cameraOpened = true;

                        webcam.startStreaming(
                                CAMERA_WIDTH,
                                CAMERA_HEIGHT,
                                OpenCvCameraRotation.UPRIGHT
                        );

                        FtcDashboard.getInstance().startCameraStream(
                                webcam,
                                CAMERA_FPS
                        );

                        System.out.println(
                                "C920 CAMERA OPENED - DASHBOARD STREAM STARTED"
                        );
                    }

                    @Override
                    public void onError(int errorCode) {

                        cameraOpened = false;

                        System.out.println(
                                "CAMERA ERROR: " + errorCode
                        );
                    }
                }
        );
    }

    /**
     * Handles automatic pickup.
     */
    public boolean updateAutoPickup(
            boolean togglePressed,
            TankDrive drive,
            Intake intake
    ) {

        if (togglePressed) {

            autoPickup = !autoPickup;

            if (!autoPickup) {
                stopPickup(drive, intake);
                return true;
            }
        }

        if (!autoPickup) {
            return false;
        }

        if (!isDetected()) {

            autoPickup = false;

            stopPickup(drive, intake);

            return true;
        }

        double offsetAngle = getOffsetAngle();

        double forwardPower;
        double turnPower = offsetAngle * TURN_MULTIPLIER;

        // If the ball is very far off to the side (> 25 deg), turn on spot first
        if (Math.abs(offsetAngle) > 25.0) {
            forwardPower = 0.0;
            double magnitude = Math.max(0.20, Math.abs(turnPower));
            turnPower = Math.copySign(magnitude, turnPower);
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));
        } else {
            // Ball in front (<= 25 deg): Drive forward AND steer toward ball simultaneously!
            // Moving forward eliminates static friction for smooth, tweak-free steering.
            forwardPower = FORWARD_POWER;
            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
        }

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(forwardPower, 0),
                        turnPower
                )
        );

        if (intake != null) {
            if (intake.intakeMotor != null) intake.intakeMotor.setPower(1.0);
            if (intake.left != null) intake.left.setPower(-1.0);
            if (intake.right != null) intake.right.setPower(1.0);
        }

        return true;
    }

    /**
     * Stops the drive and intake safely without null pointer exceptions.
     */
    private void stopPickup(TankDrive drive, Intake intake) {

        if (drive != null) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            0
                    )
            );
        }

        if (intake != null) {
            if (intake.intakeMotor != null) intake.intakeMotor.setPower(0);
            if (intake.left != null) intake.left.setPower(0);
            if (intake.right != null) intake.right.setPower(0);
            if (intake.transfer != null) intake.transfer.setPower(0);
        }
    }

    public boolean isAutoPickupActive() {
        return autoPickup;
    }

    public boolean isCameraOpened() {
        return cameraOpened;
    }

    public boolean isDetected() {
        return pipeline.isDetected();
    }

    public int getCenterX() {
        return pipeline.getCenterX();
    }

    public int getCenterY() {
        return pipeline.getCenterY();
    }

    public int getWidth() {
        return pipeline.getWidth();
    }

    public int getHeight() {
        return pipeline.getHeight();
    }

    public long getFrameCount() {
        return webcam != null ? webcam.getFrameCount() : 0;
    }

    /**
     * Calculates the horizontal angle of the detected object.
     */
    public double getOffsetAngle() {

        if (!isDetected()) {
            return 0;
        }

        double imageWidth = pipeline.getImageWidth();

        if (imageWidth <= 0) {
            return 0;
        }

        return (getCenterX() - imageWidth / 2.0)
                / imageWidth * HORIZONTAL_FOV;
    }

    /**
     * Stops the camera and automatic pickup.
     */
    public void stop() {

        autoPickup = false;

        FtcDashboard.getInstance().stopCameraStream();

        if (webcam != null && cameraOpened) {

            webcam.stopStreaming();
            webcam.closeCameraDevice();

            cameraOpened = false;
        }
    }

    /**
     * OpenCV pipeline for detecting yellow objects.
     */
    private static class YellowPipeline extends OpenCvPipeline {

        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();

        private final List<MatOfPoint> contours = new ArrayList<>();

        private volatile boolean detected = false;

        private volatile int centerX = 0;
        private volatile int centerY = 0;
        private volatile int width = 0;
        private volatile int height = 0;

        private volatile int imageWidth = CAMERA_WIDTH;
        private volatile int imageHeight = CAMERA_HEIGHT;

        @Override
        public Mat processFrame(Mat input) {

            imageWidth = input.cols();
            imageHeight = input.rows();

            Imgproc.cvtColor(
                    input,
                    hsv,
                    Imgproc.COLOR_RGB2HSV
            );

            Core.inRange(
                    hsv,
                    new Scalar(18, 100, 60),
                    new Scalar(32, 255, 255),
                    mask
            );

            contours.clear();

            Imgproc.findContours(
                    mask,
                    contours,
                    hierarchy,
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            double largestArea = 0;
            Rect largestRect = null;

            for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);

                if (area > largestArea) {

                    largestArea = area;
                    largestRect = Imgproc.boundingRect(contour);
                }

                contour.release();
            }

            contours.clear();

            if (largestRect != null && largestArea > 500) {

                detected = true;

                centerX = largestRect.x + largestRect.width / 2;
                centerY = largestRect.y + largestRect.height / 2;

                width = largestRect.width;
                height = largestRect.height;

                Imgproc.rectangle(
                        input,
                        largestRect,
                        new Scalar(0, 255, 0),
                        2
                );

                Imgproc.circle(
                        input,
                        new Point(centerX, centerY),
                        5,
                        new Scalar(255, 0, 0),
                        -1
                );

            } else {

                detected = false;

                centerX = 0;
                centerY = 0;
                width = 0;
                height = 0;
            }

            // Center alignment line.
            Imgproc.line(
                    input,
                    new Point(imageWidth / 2.0, 0),
                    new Point(imageWidth / 2.0, imageHeight),
                    new Scalar(255, 255, 255),
                    2
            );

            return input;
        }

        public boolean isDetected() {
            return detected;
        }

        public int getCenterX() {
            return centerX;
        }

        public int getCenterY() {
            return centerY;
        }

        public int getWidth() {
            return width;
        }

        public int getHeight() {
            return height;
        }

        public int getImageWidth() {
            return imageWidth;
        }
    }
}
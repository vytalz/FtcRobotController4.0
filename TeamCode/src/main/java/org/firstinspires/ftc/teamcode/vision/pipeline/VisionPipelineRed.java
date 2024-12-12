package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.tests.RedElementAlignmentFinal;
import org.firstinspires.ftc.teamcode.vision.tests.YellowElementAlignmentFinal;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionPipelineRed {

    private OpenCvCamera camera;
    private final String cameraName;
    private RedElementAlignmentFinal capstonePipeline;
    private final HardwareMap hardwareMap;
    private final int width, height;

    // Constructors for the pipeline
    public VisionPipelineRed(HardwareMap hMap, String camName) {
        hardwareMap = hMap;
        cameraName = camName;
        width = 432;
        height = 240;
    }

    public VisionPipelineRed(HardwareMap hMap, String camName, int width, int height) {
        hardwareMap = hMap;
        cameraName = camName;
        this.width = width;
        this.height = height;
    }

    // Initialize the camera and pipeline
    public void init() {
        int cameraViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraViewId);

        capstonePipeline = new RedElementAlignmentFinal();  // Initialize the yellow element detection pipeline

        camera.setPipeline(capstonePipeline);  // Set the custom pipeline
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);  // Start camera streaming
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.addGlobalWarningMessage("Warning: Camera device failed to open with EasyOpenCv error: " +
                        ((errorCode == -1) ? "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" : "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE"));
            }
        });
    }

    // Getter for camera
    public OpenCvCamera getCamera() {
        return camera;
    }

    // Set the lower color bound for yellow element detection
    public void setLowerBound(Scalar low) {
        capstonePipeline.setLowerBound(low);
    }

    // Set the upper color bound for yellow element detection
    public void setUpperBound(Scalar high) {
        capstonePipeline.setUpperBound(high);
    }

    // Set both lower and upper bounds for yellow element detection
    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        capstonePipeline.setLowerAndUpperBounds(low, high);
    }

    // Get the X position of the centroid if detected
    public double getCentroidXPosition() {
        if (capstonePipeline.getCentroid() != null) {
            return capstonePipeline.getCentroid().x;  // Return the X position of the centroid
        } else {
            return -1;  // No detection
        }
    }

    // Check if the detected object is centered within a certain threshold
    public boolean isCentered() {
        double xPosition = getCentroidXPosition();
        double centerThreshold = width * 0.05;  // Adjust the threshold as needed

        if (xPosition != -1) {
            // Check if the centroid is near the center of the frame
            double centerX = width / 2.0;
            return Math.abs(xPosition - centerX) <= centerThreshold;
        }
        return false;  // No object detected
    }

    // Getter for checking if the object is aligned in rotation
    public boolean isAligned() {
        return capstonePipeline.isAligned();
    }

    // Getter for the rotation angle
    public double getRotationAngle() {
        return capstonePipeline.getRotationAngle();
    }

}

package org.firstinspires.ftc.teamcode.vision.tests;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class YellowElementAlignmentFinal extends OpenCvPipeline {

    private Scalar low;
    private Scalar high;

    private Mat mask;
    private Mat frame;
    private Mat output;
    private Point centroid;


    private double largestContourArea;
    private double angularOffset;
    private double rotationAngle;
    private double tolerance = 5.0; // Tolerance for alignment in degrees
    private double centerTolerance = 5.0; // Tolerance for centroid alignment in pixels

    // Field of view of the camera (in degrees)
    private double cameraHorizontalFOV = 60.0;

    // Coordinates for the vertical axis (defined by Imgproc.line)
    private final int verticalAxisX = 86; // X-coordinate of the center vertical axis

    public YellowElementAlignmentFinal() {
        // Set the color range for detecting the yellow element
        low = new Scalar(0, 0, 0, 0);
        high = new Scalar(255, 255, 92.1, 255); // Adjusted upper bound for yellow
    }

    public YellowElementAlignmentFinal(Scalar lowerBound, Scalar upperBound) {
        low = lowerBound;
        high = upperBound;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame = new Mat();
        output = new Mat();
        mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);

        // Convert to YCrCb color space
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);

        // Threshold the image to get only yellow colors
        Core.inRange(output, low, high, mask);

        // Erode and dilate to remove noise
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // Draw the vertical center axis on the image
        Imgproc.line(input, new Point(verticalAxisX, input.rows()), new Point(verticalAxisX, 0), new Scalar(0, 0, 0), 1);
        Imgproc.putText(input, "Yellow", new Point(120, 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 0), 1);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            // Find the largest contour
            double maxArea = 0;
            MatOfPoint biggest = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    biggest = contour;
                }
            }

            if (biggest != null) {
                largestContourArea = maxArea;

                // Get bounding rectangle and find the rotation angle
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(biggest.toArray()));
                rotationAngle = rotatedRect.angle;

                // Adjust the angle to be relative to vertical axis
                if (rotatedRect.size.width < rotatedRect.size.height) {
                    rotationAngle = 90 + rotationAngle;
                }

                // Calculate moments to find the centroid
                Moments moments = Imgproc.moments(biggest);
                centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

                // Draw the largest contour, centroid, and rectangle on the frame
                Imgproc.drawContours(input, Arrays.asList(biggest), -1, new Scalar(255, 255, 255), 1);
                Imgproc.circle(input, centroid, 2, new Scalar(255, 255, 255), -1);

                // Draw the bounding box to visualize rotation
                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Check if the object is aligned (both rotation and center alignment)
                if (isAligned() && isCentered()) {
                    Imgproc.putText(input, "Aligned & Centered", new Point(0, 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(0, 255, 0), 1);
                } else {
                    Imgproc.putText(input, "Not Aligned", new Point(0, 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, new Scalar(0, 0, 255), 1);
                }
            }
        }

        // Release resources
        mask.release();
        output.release();

        return input;
    }

    /**
     * Check if the element is aligned with the vertical axis within a tolerance
     * @return true if the rotation angle is within the tolerance zone
     */
    public boolean isAligned() {
        return Math.abs(rotationAngle) <= tolerance || Math.abs(rotationAngle - 90) <= tolerance || Math.abs(rotationAngle - 180) <= tolerance;
    }


    /**
     * Check if the element's centroid is aligned with the vertical axis within a pixel tolerance.
     * @return true if the centroid is within the center tolerance.
     */
    public boolean isCentered() {
        // Calculate the horizontal offset between the centroid and the defined vertical axis
        double horizontalOffset = Math.abs(centroid.x - verticalAxisX);

        // Check if the horizontal offset is within the defined tolerance
        return horizontalOffset <= centerTolerance;
    }

    // Getter methods
    public Point getCentroid() {
        return centroid;
    }

    public double getLargestContourArea() {
        return largestContourArea;
    }

    public double getRotationAngle() {
        return rotationAngle;
    }

    public void setLowerBound(Scalar low) {
        this.low = low;
    }

    public void setUpperBound(Scalar high) {
        this.high = high;
    }

    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        this.low = low;
        this.high = high;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setCenterTolerance(double centerTolerance) {
        this.centerTolerance = centerTolerance;
    }

    public void setCameraHorizontalFOV(double cameraHorizontalFOV) {
        this.cameraHorizontalFOV = cameraHorizontalFOV;
    }
}
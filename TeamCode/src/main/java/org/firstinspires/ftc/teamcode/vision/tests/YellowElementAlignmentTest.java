package org.firstinspires.ftc.teamcode.vision.tests;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class YellowElementAlignmentTest extends OpenCvPipeline {

    private Scalar low;
    private Scalar high;

    private Mat mask;
    private Mat frame;
    private Mat output;
    private Point centroid;

    private double largestContourArea;
    private double angularOffset;
    private double tolerance = 2.0; // Set tolerance in degrees for "alignment"

    // Field of view of the camera (in degrees). Adjust this according to your camera specs.
    private double cameraHorizontalFOV = 60.0;

    public YellowElementAlignmentTest() {
        // Set the color range for detecting the yellow element
        low = new Scalar(0, 0, 0, 0);
        high = new Scalar(255, 255, 92.1, 255); // Adjusted upper bound for yellow
    }

    public YellowElementAlignmentTest(Scalar lowerBound, Scalar upperBound) {
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

        // Optional: Erode and dilate to remove small noise
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

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

                // Calculate moments to find the centroid
                Moments moments = Imgproc.moments(biggest);
                centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

                // Calculate angular offset from the center of the frame
                angularOffset = calculateAngularOffset(centroid.x, input.width());

                // Draw the largest contour and centroid on the frame
                Imgproc.drawContours(input, Arrays.asList(biggest), -1, new Scalar(255, 255, 255), 1);
                Imgproc.circle(input, centroid, 2, new Scalar(255, 255, 255), -1);

                // Check if the object is aligned
                if (isAligned()) {
                    Imgproc.putText(input, "Aligned", new Point(10, 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                } else {
                    Imgproc.putText(input, "Not Aligned", new Point(10, 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 1);
                }
            }
        }

        // Release resources
        mask.release();
        output.release();

        return input;
    }

    /**
     * Calculate the angular offset in degrees based on the centroid's x position
     * relative to the center of the camera's view.
     *
     * @param xCentroid    The x coordinate of the centroid
     * @param frameWidth   The width of the camera frame
     * @return The angular offset in degrees
     */
    private double calculateAngularOffset(double xCentroid, int frameWidth) {
        double centerX = frameWidth / 2.0;
        double pixelOffset = xCentroid - centerX;

        // Calculate the angular offset based on the pixel offset and the camera's horizontal field of view
        return (pixelOffset / centerX) * (cameraHorizontalFOV / 2.0);
    }

    /**
     * Check if the element is aligned within the center zone (based on tolerance)
     * @return true if the angular offset is within the tolerance zone
     */
    public boolean isAligned() {
        return Math.abs(angularOffset) <= tolerance;
    }

    // Getter methods
    public Point getCentroid() {
        return centroid;
    }

    public double getLargestContourArea() {
        return largestContourArea;
    }

    public double getAngularOffset() {
        return angularOffset;
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

    public void setCameraHorizontalFOV(double cameraHorizontalFOV) {
        this.cameraHorizontalFOV = cameraHorizontalFOV;
    }
}
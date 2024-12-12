    package org.firstinspires.ftc.teamcode.vision.tests;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RedElementDetector extends OpenCvPipeline {

    private Scalar low;
    private Scalar high;

    private Mat mask;
    private Mat frame;
    private Mat output;
    private Point centroid;

    private double largestContourArea;
    public RedElementDetector() {
        // Set the color range for red
        low = new Scalar(0, 185.6, 0, 0);
        high = new Scalar(255, 255, 117.6, 255); // Adjusted upper bound for red
    }

    public RedElementDetector(Scalar lowerBound, Scalar upperBound) {
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

        // Threshold the image to get only red colors
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

                // Draw the largest contour and centroid on the frame
                Imgproc.drawContours(input, Arrays.asList(biggest), -1, new Scalar(255, 255, 255), 1);
                Imgproc.circle(input, centroid, 2, new Scalar(255, 255, 255), -1);
            }
        }

        // Release resources
        mask.release();
        output.release();

        return input;
    }

    public Point getCentroid() {
        return centroid;
    }
    public double getLargestContourArea() {
        return largestContourArea;
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
}
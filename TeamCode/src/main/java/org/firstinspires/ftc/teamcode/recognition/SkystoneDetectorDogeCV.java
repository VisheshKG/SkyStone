package org.firstinspires.ftc.teamcode.recognition;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetectorDogeCV extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 66);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorerForLongFace = new RatioScorer(1.6, 3); // Used to find the long face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(6000,0.01); // Used to find objects near a tuned area value

    // Constants
    public static final int NUM_STONES_IN_QUARRY = 6;
    private static final int IMG_WIDTH  = 640;  // Todo: These should not be constants
    private static final int IMG_HEIGHT = 360;  // Todo: They should be extracted from camera image

    // Results of the detector
    private int quarry[] = new int[NUM_STONES_IN_QUARRY+1];
    private boolean scoring = false;            // Are we using scorers to determine the best Rectangle
    private Point screenPosition = new Point(); // Screen position of the Skystone top candidate
    private Rect foundRect = new Rect();        // Found rect
    private ArrayList<Rect> stoneRects = new ArrayList<>(); // Rectangles containing Stone candidates
    private ArrayList<Rect> skystoneRects = new ArrayList<>(); // Rectangles containing Skystone candidates

    // history counters and variables
    private int currentCount = 0;                    // number of rectangle detection pass
    private int reject[] = new int[4];


    // Attributes of the objects of interest to be recognized in the image.
    private int minRectArea     = 0;
    private int maxRectArea     = Integer.MAX_VALUE;

    // image processing canvases
    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    public void setTargetAreaSize(int min, int max) {
        minRectArea = min;
        maxRectArea = max;
    }

    public Point getRectanglePosition() {
        return screenPosition;
    }

    public Rect foundRectangle() {
        return foundRect;
    }

    public ArrayList<Rect> getStoneCandidates() {
        return stoneRects;
    }

    public ArrayList<Rect> getSkystoneCandidates() {
        return skystoneRects;
    }

    public int  getRectangleDetectionCount() { return currentCount; }

    public int[] getStoneQuarryCounts() { return quarry; }

    public int[] getRejectCounts() { return reject; }

    public SkystoneDetectorDogeCV() {
        detectorName = "Skystone Detector";
    }

    @Override
    public Mat process(Mat input) {

        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);

        // Detect YELLOW color objects by detecting contours around yellow areas in the images
        yellowFilter.process(workingMat.clone(), yellowMask);
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(255,30,30),2); // Draw contours in red

        // Current result
        Rect bestRect = foundRect;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        // we want to allocate new memory since the stoneRects and skystoneRects is returned
        // outside this class and is used in different thread than the image proc pipeline
        ArrayList<Rect> rectanglesYellow = new ArrayList<>();
        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            // Draw rectangle on monitor display
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect in blue
            // Record if it meets the target area size
            if ((rect.area() > minRectArea) && (rect.area() <= maxRectArea)) {
                rectanglesYellow.add(rect);
            }

            if (scoring) {
                double score = calculateScore(cont); // Get the difference score using the scoring API
                // If the result is better then the previously tracked one, set this rect as the new best
                if (score < bestDifference) {
                    bestDifference = score;
                    bestRect = rect;
                }
            }
        }
        stoneRects = rectanglesYellow;

        // ??? Why are we drawing a white rectangle in the black mask, around the best Yellow colored stone?
        Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255,255,255), 1, Imgproc.LINE_4, 0);

        // Detect BLACKish color objects by detecting contours around black/gray areas in the images
        blackFilter.process(workingMat.clone(), blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();
        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(0,255,0),2); // Draw contours in green

        // we want to allocate new memory since the stoneRects and skystoneRects is returned
        // outside this class and is used in different thread than the image proc pipeline
        ArrayList<Rect> rectanglesBlack = new ArrayList<>();
        for(MatOfPoint cont : contoursBlack){
            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            // Draw rectangle on monitor display
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(112,48,160),2); // Draw rect in purple
            // Record if it meets the target area size
            if ((rect.area() > minRectArea) && (rect.area() <= maxRectArea)) {
                rectanglesBlack.add(rect);
            }

            if (scoring) {
                double score = calculateScore(cont); // Get the difference score using the scoring API
                // If the result is better then the previously tracked one, set this rect as the new best
                if (score < bestDifference) {
                    bestDifference = score;
                    bestRect = rect;
                }
            }
        }
        skystoneRects = rectanglesBlack;

        if (scoring && (bestRect != null)) {
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(0,255,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        }
        else {
            found = false;
        }

        // Analyse the array of Black rectangles to identify 1 Skystone in Position 6, 5, 4
        this.calculateSkystoneLocation();

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorerForLongFace);

        // Add different scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }

    protected void calculateSkystoneLocation() {

        for (Rect rect : skystoneRects) {
            // skystone height / width ratio must be within a range around 1.6
            double w = rect.width;
            double h = rect.height;
            double ratio = Math.max(Math.abs(h / w), Math.abs(w / h)); // We use max in case h and w get swapped due to image rotation
            if (ratio < 1.4 || ratio > 1.8) {
                reject[1]++;
                continue;
            }
            // the bottom of rectangle (x value) should be close to 66% of the image dimension
            if (Math.abs(rect.x - (IMG_HEIGHT * 0.66)) > 40) {
                reject[2]++;
                continue;
            }

            // ignore extraneous object images on the sides
            int stoneWidth = IMG_WIDTH * 3 / 16;
            int leftLimit = IMG_WIDTH / 4; // first stone starts at approx at 160 out of 640 pixels wide image
            leftLimit -= (stoneWidth / 2);   // move leftLimit by half of stone width to center of stone
            if (rect.y < (leftLimit)) {
                reject[3]++;
                continue;
            }
            // if we still have a rectangle then it meets all requirements of skystone in quarry
            // calculate position (value = 0, 1, 2 or 3)
            int pos = (rect.y - leftLimit) / stoneWidth;
            // stone position in the quarry is counted from the audience wall starting with 1, reverse the count
            pos = NUM_STONES_IN_QUARRY - pos;

            // skystone detected at position pos in the quarry, increment count for that position
            quarry[pos]++;
        }
        // increment the process counter for this iteration
        currentCount++;
    }

    public int getSkystoneLocationInQuarry() {
        int largest = 0;
        int pos = 4; // actual value will be set in loop below, default value here if loop fails
        for (int i = 6; i > 3; i--) {
            if (quarry[i] > largest) {
                largest = quarry[i];
                pos = i;
            }
        }
        return pos;
    }
}

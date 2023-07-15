package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⠻⢿⣿⣿⣿⣿⣿⣿⠟⠀⠈⣻⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣄⠀⠀⠈⠛⢿⣿⡿⠁⠀⣠⡾⢋⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣀⠙⠻⣦⣄⡀⠀⠈⠛⢶⣴⡋⣠⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣀⣽⠿⢶⣄⡀⠀⠈⠛⢿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠀⢀⣿⣿⠷⣤⣀⠀⠈⠙⢿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⣴⡿⢉⣿⣄⡈⠙⠷⣦⣴⡿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠁⠀⣠⣾⠋⣴⣿⣿⣿⣿⣶⣤⡀⣿⣼⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⣿⣿⠋⠀⢀⣴⠟⣡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⣠⡿⢋⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣿⣿⡟⠁⠀⢀⣾⠟⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⡿⠋⠀⠀⣴⡿⢡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⡁⠀⣠⣾⢋⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⡏⠛⣿⠟⣵⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 ⣿⣿⣿⣶⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
*/

public class SignalPipeline extends OpenCvPipeline {
    // april tag detector native object
    private long detectorPtr;

    // array of targets to search for
    public volatile int[] detectionIds = new int[3];

    // array of rect coords to crop image
    public int[] cropRect = null;

    // output code:
    // -1 : no target found
    // 0 : detectionIds[0] found
    // ...
    public volatile int targetFound = -1;

    // flag to signal end of april tag detection
    private boolean killAprilTags = false;
    private boolean startAprilTags = false;

    public SignalPipeline(float _initDecimation, int[] _targets, int[] _cropRectCoords){
        super();

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, _initDecimation, 2);
        System.arraycopy(_targets, 0, this.detectionIds, 0, 3);
        this.cropRect = new int[4];
        System.arraycopy(_cropRectCoords, 0, this.cropRect, 0, 4);

        if(this.detectorPtr == 0){
            killAprilTags = true;
        }
    }

    public SignalPipeline(float _initDecimation, int[] _targets) {
        super();

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, _initDecimation, 2);
        System.arraycopy(_targets, 0, this.detectionIds, 0, 3);

        // something failed at april tag detector initialization in the native fn
        if (this.detectorPtr == 0) {
            killAprilTags = true;
        }
    }

    double[] rectCenter;
    double rectX, rectY;
    int rectDiameter = 30;
    Rect rect = new Rect(0,0,30,30);
    Scalar green = new Scalar(0, 255, 0);
    Mat colorStream = new Mat();
    String detectionText = "";
    Point textOrigin = new Point(100, 100);
    int font = Imgproc.FONT_HERSHEY_SIMPLEX;
    Scalar red = new Scalar(255, 0, 0);

    private void detectAprilTag(Mat _input) {
        if (_input.empty()) {
            this.targetFound = -1;
            return;
        }

        _input.copyTo(colorStream);
        if(this.cropRect != null){
            Rect roi = new Rect(cropRect[0], cropRect[1], cropRect[2], cropRect[3]);
            _input = _input.submat(roi);
        }
        Imgproc.cvtColor(_input, _input, Imgproc.COLOR_RGB2GRAY);

        long detectionsPtrToArr = 0; // pointer to array
        long[] detectionsArrOfPtrs = new long[]{0}; // array of pointers
        boolean fail = false;

        try {
            // run detection
            detectionsPtrToArr = AprilTagDetectorJNI.runApriltagDetector(this.detectorPtr, _input.nativeObj);

            // convert native array of detections to java array of native objects
            detectionsArrOfPtrs = ApriltagDetectionJNI.getDetectionPointers(detectionsPtrToArr);
        } catch (IllegalArgumentException e) {
            fail = true;
            e.printStackTrace();
        }

        if (fail) {
            this.targetFound = -1;
            return;
        }

        // iterate through the detection array
        for (int i = 0; i < detectionsArrOfPtrs.length; i++) {
            int id = -1; // current element tag id

            try {
                // get current id
                id = ApriltagDetectionJNI.getId(detectionsArrOfPtrs[i]);
                rectCenter = ApriltagDetectionJNI.getCenterpoint(detectionsArrOfPtrs[i]);
                rect.x = (int)rectCenter[0] - (rectDiameter / 2);
                rect.y = (int)rectCenter[1] - (rectDiameter / 2);
                Imgproc.rectangle(colorStream, rect, green, 2);
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }

            // compare id to each target in _detectionIds[]
            if (id == this.detectionIds[0]) {
                this.targetFound = 0;
                detectionText = "Sleeve 1";
            } else if (id == this.detectionIds[1]) {
                this.targetFound = 1;
                detectionText = "Sleeve 2";
            } else if (id == this.detectionIds[2]) {
                this.targetFound = 2;
                detectionText = "Sleeve 3";
            } else {
                detectionText = "No Sleeve";
            }
        }

        try {
            // cleanup (these are dynamically allocated C arrays, so this has to be done)
            ApriltagDetectionJNI.freeDetectionList(detectionsPtrToArr);
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        }

        // Add the text
        Imgproc.putText(colorStream, detectionText, textOrigin, font, 1, red);

        colorStream.copyTo(_input); // restore the color to show to user
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!killAprilTags && startAprilTags) {
            detectAprilTag(input);
        }
        return input;
    }

    public void startAprilTagDetection() {
        startAprilTags = true;
        killAprilTags = false;
    }


    public void stopAprilTagDetection() {
        if (killAprilTags) {
            return;
        }
        killAprilTags = true;
        startAprilTags = false;
//       try {
//           AprilTagDetectorJNI.releaseApriltagDetector(this.detectorPtr);
//       } catch (IllegalArgumentException e){
//           e.printStackTrace();
//       }
    }
}


package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
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

public class AprilTagsPipeline extends OpenCvPipeline {
    // april tag detector native object
    private long detectorPtr;

    // array of pointers to detections
    private long[] _detectionsPtr;

    // array of targets to search for
    public volatile int[] _detectionIds = new int[3];

    // array of found targets
    public volatile int[] __ids = new int[3];

    // output code:
    // -1 : no target found
    // 0 : _detectionIds[0] found
    // ...
    public volatile int targetFound = -1;

    // holds unknown errors
    public Exception _e;

    // high decimation -> low distance
    // low decimation -> high distance
    // 3 -> approx 1 meter
    private float decimation;

    // flag to change decimation synchronously
    private boolean _chDecimation = false;

    // locks used for synchronization (might not even be needed)
    private Object lock1 = new Object();
    private Object lock2 = new Object();

    // flag to signal end of detection
    public boolean killThis = false;

    public AprilTagsPipeline(float initDecimation, int[] targets) {
        super();
        this.decimation = initDecimation;

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, this.decimation, 2);
        System.arraycopy(targets, 0, this._detectionIds, 0, 3);

        // something failed at april tag detector initialization in the native fn
        if (this.detectorPtr == 0) {
            killThis = true;
            this.kill();
        }
    }

    // no need for this - might delete later
    public void setTargets(int[] targets) { // praying this works
        synchronized (this.lock2) {
            System.arraycopy(targets, 0, this._detectionIds, 0, 3);
        }
    }

    public float getDecimation() {
        return this.decimation;
    }

    public void setDecimation(float _decimation) {
        this.decimation = _decimation;
        this._chDecimation = true;
    }

    @Override
    public Mat processFrame(Mat input) {
        // if this is killed just return input
        if (killThis) {
            return input;
        }

        // prevent future IllegalArgumentExceptions
        if (input.empty()) {
            return input;
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        // set decimation synchronously if needed
        synchronized (this.lock1) {
            if (this._chDecimation) {
                boolean stillNeededToChDecimation = false;

                try {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(this.detectorPtr, this.decimation);
                } catch (IllegalArgumentException e) {
                    stillNeededToChDecimation = true;
                    // make sure it retries to change decimation
                }

                this._chDecimation = stillNeededToChDecimation;
            }
        }

        // pointer to C array of detections
        long _detections = 0;

        try {
            // run the detection
            _detections = AprilTagDetectorJNI.runApriltagDetector(this.detectorPtr, input.nativeObj);

            // convert native array of detections to java array of native objects
            this._detectionsPtr = ApriltagDetectionJNI.getDetectionPointers(_detections);
        } catch (IllegalArgumentException e) {
            ;
            ; // yeet errors
        }

        // compare detected tags to targets synchronously
        synchronized (this.lock2) {

            // make sure detection array is not null
            if (this._detectionsPtr == null) {
                return input;
            }

            // iterate through the detection array
            for (int i = 0; i < this._detectionsPtr.length; i++) {
                int id = -1; // current element tag id

                try {
                    // get current detecetion id
                    id = ApriltagDetectionJNI.getId(this._detectionsPtr[i]);
                } catch (IllegalArgumentException e) {
                    ;
                    ; // yeet the error
                }

                // store first 3 detected tags inside __ids[]
                if (i < 2) {
                    this.__ids[i] = id;
                }

                // compare id to each target in _detectionIds[]
                if (id == this._detectionIds[0]) {
                    this.targetFound = 0;
                } else if (id == this._detectionIds[1]) {
                    this.targetFound = 1;
                } else if (id == this._detectionIds[2]) {
                    this.targetFound = 2;
                }

                // if one tag is found flag to kill this
                if (id >= 0) {
                    killThis = true;
                }
            }
        }

        try {
            // cleanup (these are dynamically allocated C arrays, so this has to be done)
            ApriltagDetectionJNI.freeDetectionList(_detections);
        } catch (IllegalArgumentException e) {
            ;
            ;
        }

        return input;
    }


    public void kill() {
        try {
            AprilTagDetectorJNI.releaseApriltagDetector(this.detectorPtr);
        } catch (IllegalArgumentException e) {
            ;
            ; // yeet the error
        }
    }
}


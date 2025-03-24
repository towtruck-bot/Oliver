package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.LinkedList;

// This is a crappy bundle class
class ConfigThresholdValue {
    private final Scalar val;

    public ConfigThresholdValue(String name, int v0, int v1, int v2) {
        val = new Scalar(v0, v1, v2);

        FtcDashboard.getInstance().addConfigVariable(BlockDetectionPipeline.class.getName(), name + "0", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return (int) val.val[0];
            }

            @Override
            public void set(Integer value) {
                val.val[0] = value.doubleValue();
            }
        });
        FtcDashboard.getInstance().addConfigVariable(BlockDetectionPipeline.class.getName(), name + "1", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return (int) val.val[1];
            }

            @Override
            public void set(Integer value) {
                val.val[1] = value.doubleValue();
            }
        });
        FtcDashboard.getInstance().addConfigVariable(BlockDetectionPipeline.class.getName(), name + "2", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return (int) val.val[2];
            }

            @Override
            public void set(Integer value) {
                val.val[2] = value.doubleValue();
            }
        });
    }

    public Scalar getScalar() {
        return val;
    }
}

@Config
public class BlockDetectionPipeline extends OpenCvPipeline {
    public enum BlockColor {
        RED,
        BLUE,
        YELLOW
    }

    private final boolean verbose;
    private int pp1 = 0, pp2 = 1;
    private Mat[] post = new Mat[2]; // Postprocessing
    private Mat boundingRects;
    private Mat mask;
    private long s = 0;
    private TelemetryPacket packet;
    private final ConfigThresholdValue redL;
    private final ConfigThresholdValue redH;
    private final ConfigThresholdValue blueL;
    private final ConfigThresholdValue blueH;
    private final ConfigThresholdValue yellowL;
    private final ConfigThresholdValue yellowH;
    private BlockColor b;
    enum ReturnVal {
        MASK,
        INPUT,
        RECT
    }
    public static ReturnVal returnVal = ReturnVal.INPUT;
    private Size imageSize;
    public static double processingSizeMul = 0.5;
    private LinkedList<MatOfPoint> detections = new LinkedList<>();
    private Size processingSize;
    private Mat trashbuf = new Mat();
    private MatOfPoint2f rotated = new MatOfPoint2f();

    // Detected values
    private Point center = new Point();
    private double angle = 0;

    public BlockDetectionPipeline(BlockColor b, boolean verbose) {
        this.verbose = verbose;
        if (verbose)
            packet = new TelemetryPacket();

        redL = new ConfigThresholdValue("redL", 30, 168, 70);
        redH = new ConfigThresholdValue("redH", 130, 255, 170);
        blueL = new ConfigThresholdValue("blueL", 0, 0, 0);
        blueH = new ConfigThresholdValue("blueH", 0, 0, 0);
        yellowL = new ConfigThresholdValue("yellowL", 0, 0, 0);
        yellowH = new ConfigThresholdValue("yellowH", 0, 0, 0);

        this.b = b;
    }

    public BlockDetectionPipeline(BlockColor b) {
        this(b, false);
    }

    @Override
    public void init(Mat ff) {
        imageSize = ff.size();
        processingSize = new Size((int) (imageSize.width * processingSizeMul), (int) (imageSize.height / processingSizeMul));
        post = new Mat[] {
            new Mat(processingSize, CvType.CV_16F),
            new Mat(processingSize, CvType.CV_16F)
        };
        mask = new Mat(processingSize, CvType.CV_16F);
        boundingRects = new Mat(processingSize, CvType.CV_16F);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Postprocessing
        s = System.currentTimeMillis();

        Imgproc.resize(input, post[pp1], imageSize, processingSizeMul, processingSizeMul, Imgproc.INTER_LINEAR);
        Imgproc.cvtColor(post[pp1], post[pp2], Imgproc.COLOR_RGB2YCrCb);
        switchBuffers();

        switch (b) {
            case RED:
                Core.inRange(post[pp1], redL.getScalar(), redH.getScalar(), post[pp2]);
                break;

            case BLUE:
                Core.inRange(post[pp1], blueL.getScalar(), blueH.getScalar(), post[pp2]);
                break;

            case YELLOW:
                Core.inRange(post[pp1], yellowL.getScalar(), yellowH.getScalar(), post[pp2]);
                break;
        }
        switchBuffers();

        if (returnVal == ReturnVal.MASK)
            post[pp1].copyTo(mask);

        log("Postprocess time", (System.currentTimeMillis() - s) + "ms");

        s = System.currentTimeMillis();

        // Now get the largest blob
        detections.clear();
        Imgproc.findContours(post[pp1], detections, trashbuf, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect largest = null;
        MatOfPoint largestContour = null;
        for (MatOfPoint c : detections) {
            Rect r = Imgproc.boundingRect(c);

            if (largest == null || r.area() > largest.area()) {
                largest = r;
                largestContour = c;
            }
        }

        if (largest != null) {
            rotated.fromArray(largestContour.toArray());
            RotatedRect r = Imgproc.minAreaRect(rotated);
            center = r.center;
            angle = r.angle;

            log("Block angle", angle + "");
            log("Block x", center.x + "");
            log("Block y", center.y + "");
            RobotLog.i("UNIQUE MESSAGE " + angle);
        }

        log("Data extract time", + (System.currentTimeMillis() - s) + "ms");

        if (returnVal == ReturnVal.RECT) {
            input.copyTo(boundingRects);
            if (largest != null) {
                Imgproc.rectangle(boundingRects, largest, new Scalar(255, 0, 0), 2);
                Imgproc.circle(boundingRects, center, 5, new Scalar(0, 255, 0));
            }
        }

        if (verbose) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }


        switch (returnVal) {
            case INPUT:
            default:
                return input;
            case MASK:
                return mask;
            case RECT:
                return boundingRects;
        }
    }

    private void log(String key, String text) {
        if (verbose)
            packet.put("[VISION::BlockDetectionPipeline] " + key, text);
    }

    private void switchBuffers() {
        pp1 ^= 1;
        pp2 ^= 1;
    }
}
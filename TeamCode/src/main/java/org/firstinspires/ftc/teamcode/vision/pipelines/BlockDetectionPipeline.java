package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

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
                return (int) val.val[0];
            }

            @Override
            public void set(Integer value) {
                val.val[0] = value.doubleValue();
            }
        });
        FtcDashboard.getInstance().addConfigVariable(BlockDetectionPipeline.class.getName(), name + "2", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return (int) val.val[0];
            }

            @Override
            public void set(Integer value) {
                val.val[0] = value.doubleValue();
            }
        });
    }

    public Scalar getScalar() {
        return val;
    }
}

@Config
public class BlockDetectionPipeline extends OpenCvPipeline {
    enum BlockColor {
        RED,
        BLUE,
        YELLOW
    }

    private final boolean verbose;
    private int pp1 = 0, pp2 = 1;
    private Mat[] post = new Mat[2]; // Postprocessing
    private Mat mask = new Mat();
    private long s = 0;
    private TelemetryPacket packet;
    private ConfigThresholdValue redL;
    private ConfigThresholdValue redH;
    private ConfigThresholdValue blueL;
    private ConfigThresholdValue blueH;
    private ConfigThresholdValue yellowL;
    private ConfigThresholdValue yellowH;
    private BlockColor b;
    enum ReturnVal {
        MASK,
        INPUT,
        RECT
    }
    public static ReturnVal returnVal = ReturnVal.INPUT;

    public BlockDetectionPipeline(boolean verbose, BlockColor b) {
        this.verbose = verbose;
        if (verbose)
            packet = new TelemetryPacket();

        redL = new ConfigThresholdValue("redL", 0, 0, 0);
        redH = new ConfigThresholdValue("redH", 0, 0, 0);
        blueL = new ConfigThresholdValue("blueL", 0, 0, 0);
        blueH = new ConfigThresholdValue("blueH", 0, 0, 0);
        yellowL = new ConfigThresholdValue("yellowL", 0, 0, 0);
        yellowH = new ConfigThresholdValue("yellowH", 0, 0, 0);

        this.b = b;
    }

    public BlockDetectionPipeline(BlockColor b) {
        this(false, b);
    }

    @Override
    public void init(Mat ff) {
        post = new Mat[] {
            new Mat(),
            new Mat()
        };
    }

    @Override
    public Mat processFrame(Mat input) {
        // Postprocessing
        s = System.currentTimeMillis();

        input.convertTo(post[pp1], CvType.CV_8U);
        Imgproc.resize(post[pp1], post[pp2], new Size(320, 240));
        switchBuffers();
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
            mask = post[pp1].clone();

        // Now get the largest blob
        LinkedList<MatOfPoint> detected = new LinkedList<>();
        // Imgproc.findContours(post[pp1], ); TODO

        log("Postprocess frame time: " + (System.currentTimeMillis() - s) + "ms");

        switch (returnVal) {
            case INPUT:
            case RECT:
            default:
                return input;
            case MASK:
                return mask;
        }
    }

    private void log(String text) {
        if (verbose)
            packet.put("[VISION::BlockDetectionPipeline]", text);
    }

    private void switchBuffers() {
        if (pp1 == 1) {
            pp1--;
            pp2++;
        } else {
            pp1++;
            pp2--;
        }
    }
}
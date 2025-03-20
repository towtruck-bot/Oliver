package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// c = Custom ;-;
// Make threshold all in 1 sigma
/*class ThresholdValueProvider<T> implements ValueProvider<T> {
    private T value1

    public cValueProvider(T defaultValue) {
        value = defaultValue;
    }

    @Override
    public T get() {
        return value;
    };

    @Override
    public void set(T value) {
        this.value = value;
    }
}

@Config
public class BlockDetectionPipeline extends OpenCvPipeline {
    private final cValueProvider<Integer> redY = new cValueProvider<>(255);
    private final cValueProvider<Integer> redCr = new cValueProvider<>(255);
    private final cValueProvider<Integer> redCb = new cValueProvider<>(255);

    private Mat post = new Mat(); // Postprocessing
    private Mat redHold = new Mat();
    private Mat blueHold = new Mat();
    private Mat yellowHold = new Mat();

    @Override
    public void init(Mat ff) {
        FtcDashboard.getInstance().addConfigVariable(getClass().getName(), "redY", redY);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, post, Imgproc.COLOR_RGB2YCrCb);


        Core.inRange();
        return input;
    }
}*/

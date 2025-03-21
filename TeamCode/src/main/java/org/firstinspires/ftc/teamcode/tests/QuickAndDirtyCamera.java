package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipelines.BlockDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class QuickAndDirtyCamera extends LinearOpMode {
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.getPackageName()
        );

        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"),
            cameraMonitorViewId
        );
        camera.setPipeline(new BlockDetectionPipeline(BlockDetectionPipeline.BlockColor.RED, true));

        waitForStart();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.e("[VISION] " + " Error code: " + errorCode);
            }
        });

        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("[VISION] FPS", camera.getFps());
            packet.put("[VISION] Overhead (ms)", camera.getOverheadTimeMs());
            packet.put("[VISION] Pipeline (ms)", camera.getPipelineTimeMs());
            packet.put("[VISION] Total (ms)", camera.getTotalFrameTimeMs());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}

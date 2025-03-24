package org.firstinspires.ftc.teamcode.vision.tests;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.List;

@TeleOp
public class LimelightTest extends LinearOpMode {
    private Limelight3A limelight;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            telemetry.setMsTransmissionInterval(11);

            limelight.pipelineSwitch(1);
            /*
             * Starts polling for data.
             */
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

            limelight.start();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double x = colorTarget.getTargetXDegrees(); // Where it is (left-right)
                    double y = colorTarget.getTargetYDegrees(); // Where it is (up-down)
                    double area = colorTarget.getTargetArea(); // size (0-100)
                    List<List<Double>> targetCorners = colorTarget.getTargetCorners();
                    telemetry.addData("X Degrees", x);
                    telemetry.addData("Y Degrees", y);
                    telemetry.addData("area", area);
                    telemetry.addData("target corners", targetCorners);
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.update();
        }
    }
}

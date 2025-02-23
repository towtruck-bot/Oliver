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
            limelight.start();
            LLResult result = limelight.getLatestResult();
            if (result !=null) {
                if (result.isValid()) {
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);


                }
            }
            else
                telemetry.addData("result:  ", "null");

            telemetry.update();
        }
    }
}

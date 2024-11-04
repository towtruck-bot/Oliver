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

            limelight.pipelineSwitch(0);
            /*
             * Starts polling for data.
             */
            limelight.start();
            LLResult result = limelight.getLatestResult();
            /*if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    Log.d("", "result: " + result.getTx());
                }
                else {
                    //Log.e("", ""+limelight.isConnected());
                    //Log.e("", "" +limelight.getStatus());
                    Log.e("", "result is not valid");
                }
            }
            else
                Log.e("", "null");

             */

            // print some data for each detected target
            if (result !=null) {
                if (result.isValid()) {
                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }
            }
            else
                telemetry.addData("result: ", "null");

            telemetry.update();
        }
    }
}

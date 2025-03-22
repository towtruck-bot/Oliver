package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LLBlockDetectionTest extends LinearOpMode {
    private Limelight3A ll;

    @Override
    public void runOpMode() throws InterruptedException {
        ll = hardwareMap.get(Limelight3A.class, "limelight");

        ll.pipelineSwitch(0);
        waitForStart();
        ll.start();

        for (int i = 0; i < 5 && opModeIsActive(); i++) {
            long start = System.currentTimeMillis();
            ll.pipelineSwitch(0);
            Log.e("LIMELIGHT", (System.currentTimeMillis() - start) + "");
            start = System.currentTimeMillis();
            ll.pipelineSwitch(1);
            Log.e("LIMELIGHT", (System.currentTimeMillis() - start) + "");
        }

    }
}

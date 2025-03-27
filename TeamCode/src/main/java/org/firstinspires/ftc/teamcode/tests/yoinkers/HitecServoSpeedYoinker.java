package org.firstinspires.ftc.teamcode.tests.yoinkers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HitecServoSpeedYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Servo s = hardwareMap.get(Servo.class, "turretRotation");
        hardwareMap.get(Servo.class, "turretArm").setPosition(0.5);

        waitForStart();
        Log.e(getClass().getSimpleName(), 0.2966648 * 0 + " is position 0");
        Log.e(getClass().getSimpleName(), 0.2966648 * 0.6 + " is position 0.6");

        long start = System.currentTimeMillis();
        boolean p = false;
        while (opModeIsActive()) {
            if (System.currentTimeMillis() - start > 1000) {
                p = !p;
                start = System.currentTimeMillis();
            }

            if (p)
                s.setPosition(0);
            else
                s.setPosition(0.6);
        }
    }
}

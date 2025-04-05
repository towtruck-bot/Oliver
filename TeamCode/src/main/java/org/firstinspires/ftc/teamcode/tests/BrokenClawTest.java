package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@TeleOp
public class BrokenClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Servo s = hardwareMap.get(Servo.class, "depositClaw");

        boolean b = false;
        long start = System.currentTimeMillis();
        while (opModeIsActive()) {
            s.setPosition(b ? 0 : 1);
            if (System.currentTimeMillis() - start >= 250) {
                start = System.currentTimeMillis();
                b = !b;
            }
            //hq.update();
        }
    }
}

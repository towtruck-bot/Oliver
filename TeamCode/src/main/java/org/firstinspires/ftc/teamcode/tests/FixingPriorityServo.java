package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@TeleOp
@Config
public class FixingPriorityServo extends LinearOpMode {
    public static double pose = 0;
    public static double power = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        PriorityServo servo = new PriorityServo(
                hardwareMap.get(Servo.class, "testServo"),
                "testServo",
                PriorityServo.ServoType.AXON_MINI,
                1,
                0, 1,
                0,
                false,
                1, 2
        );
        HardwareQueue hardwareQueue = new HardwareQueue();
        hardwareQueue.addDevice(servo);

        waitForStart();

        while (opModeIsActive()) {
            Globals.START_LOOP();
            servo.setTargetPose(pose, power);
            hardwareQueue.update();
        }
    }
}

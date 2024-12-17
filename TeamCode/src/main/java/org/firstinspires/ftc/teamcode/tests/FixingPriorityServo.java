package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@TeleOp
@Config
public class FixingPriorityServo extends LinearOpMode {
    public static double pose = 0;
    public static double power = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        nPriorityServo servo = new nPriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "intakeFlipServo")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0,
                1,
                0,
                new boolean[] {false},
                1,
                2
        );
        HardwareQueue hardwareQueue = new HardwareQueue();
        hardwareQueue.addDevice(servo);

        waitForStart();

        while (opModeIsActive()) {
            Globals.START_LOOP();
            servo.setTargetPos(pose, power);
            hardwareQueue.update();
        }
    }
}

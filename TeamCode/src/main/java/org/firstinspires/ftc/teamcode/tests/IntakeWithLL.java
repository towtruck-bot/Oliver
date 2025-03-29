package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp
@Config
public class IntakeWithLL extends LinearOpMode {
    public static double extensionLength = 20;
    public static boolean restart = false;
    public static boolean grab = false;
    public static boolean setGrab = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.nclawIntake.useCamera(true);
        robot.nclawIntake.extend();
        while (opModeIsActive()) {
            robot.vision.setOffset(new Vector2(robot.nclawIntake.getIntakeRelativeToRobot(), 0));
            Pose2d blockPos = robot.vision.getBlockPos();

            TelemetryUtil.packet.put("blockPos.x", blockPos.x);
            TelemetryUtil.packet.put("blockPos.y", blockPos.y);
            TelemetryUtil.packet.put("blockPos.heading", blockPos.heading);

            if (restart) {
                robot.vision.startDetection();
                restart = false;
            }

            if (setGrab) {
                robot.nclawIntake.setTargetPose(blockPos);
                robot.nclawIntake.grab(grab);
                setGrab = false;
            }

            robot.nclawIntake.setIntakeLength(extensionLength);

            robot.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp
@Config
public class IntakeWithLL extends LinearOpMode {
    public static double extensionLength = 10;
    public static boolean setExtend = false;
    public static boolean setRetract = false;
    public static boolean setGrab = false;
    public static boolean grab = false;
    public static boolean transferRequest = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        robot.nclawIntake.resetExtendoEncoders();

        robot.nclawIntake.useCamera(true);
        //robot.nclawIntake.extend();
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        ButtonToggle x_b = new ButtonToggle();
        ButtonToggle y_b = new ButtonToggle();
        ButtonToggle a_b = new ButtonToggle();

        while (opModeIsActive()) {
            robot.vision.setOffset(new Vector2(robot.nclawIntake.getIntakeRelativeToRobot(), 0));
            Pose2d blockPos = robot.vision.getBlockPos();

            TelemetryUtil.packet.put("blockPos.x", blockPos.x);
            TelemetryUtil.packet.put("blockPos.y", blockPos.y);
            TelemetryUtil.packet.put("blockPos.heading", blockPos.heading);
            RobotLog.e("blockPos.heading " + Math.toDegrees(blockPos.heading));

            Canvas c = TelemetryUtil.packet.fieldOverlay();
            c.setStroke("#444400");
            c.strokeCircle(blockPos.x, blockPos.y, 3);
            c.strokeLine(blockPos.x, blockPos.y, blockPos.x + 5 * Math.sin(blockPos.heading), blockPos.y + 5 * Math.cos(blockPos.heading));

            if (y_b.isClicked(gamepad1.y))
                robot.nclawIntake.extend();

            if (x_b.isClicked(gamepad1.x))
                robot.ndeposit.startSampleDeposit();

            if (a_b.isClicked(gamepad1.a))
                robot.ndeposit.deposit();

            /*if (setExtend) {
                robot.vision.startDetection();
                robot.nclawIntake.extend();
                setExtend = false;
            }

            if (setRetract) {
                robot.vision.stopDetection();
                robot.nclawIntake.retract();
                setRetract = false;
            }

            if (transferRequest) {
                robot.ndeposit.startTransfer();
                transferRequest = false;
            }*/

            if (robot.nclawIntake.isTransferReady() && robot.ndeposit.isTransferReady()) {
                robot.nclawIntake.finishTransfer();
                robot.ndeposit.finishTransfer();
            }

            /*if (setGrab) {
                robot.nclawIntake.setTargetPose(new Pose2d(
                        blockPos.x,
                        blockPos.y,
                        AngleUtil.mirroredClipAngleTolerence(-blockPos.heading, Math.toRadians(20))
                ));
                robot.nclawIntake.grab(grab);
                setGrab = false;
            }*/

            TelemetryUtil.packet.put("velocityLowPass", robot.vision.getVelocityLowPass());
            robot.nclawIntake.setIntakeLength(extensionLength);
            robot.drivetrain.drive(gamepad1);

            robot.update();
        }
    }
}

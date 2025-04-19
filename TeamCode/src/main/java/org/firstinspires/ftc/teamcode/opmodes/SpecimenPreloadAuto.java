package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Autonomous(name = "A. Specimen Auto", preselectTeleOp = "A. Teleop")
@Config
public class SpecimenPreloadAuto extends LinearOpMode {
    private Robot robot;

    public static double[] dX = {-6.0, -8.0, -4.0, -2.0, 0.0, 2.0, 4.0};
    public static double dY = 31.85;

    public static double px = -60, py = 48, cy = 58, mx = -28, my = 48;

    public static double b1x = -50.4, b1y = 27.5;
    public static double b2x = -60.3, b2y = 27.5;
    public static double b3x = -69.7, b3y = 27.5;
    public static double ix = -40, iy = 61.8, sx = 0, sy = 48;

    public static boolean useIntake = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.isRed = false;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = true;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(() -> !isStopRequested());

        robot.sensors.resetPosAndIMU();

        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_TARGET);
        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);

        robot.ndeposit.state = nDeposit.State.HOLD;
        robot.ndeposit.presetDepositHeight(true, true, false);

        // TODO: Implement to emulate goToPointWIthIntake. Use vision? or dynamic intakeAt
        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_TARGET);
        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setRetryGrab(true);

        while (opModeInInit() && !isStopRequested()) {
            robot.sensors.setOdometryPosition(-0.5 * Globals.ROBOT_WIDTH, 70.5 - Globals.ROBOT_REVERSE_LENGTH, Math.toRadians(270));
            robot.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;
        robot.update();

        Globals.autoStartTime = System.currentTimeMillis();

        // Score preload
        robot.ndeposit.startSpecimenDeposit();
        robot.drivetrain.goToPoint(
                new Pose2d(dX[0], dY, Math.toRadians(270)),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.isBusy());

        robot.ndeposit.deposit();
        robot.drivetrain.goToPoint(
                new Pose2d(sx, sy, -Math.PI / 2),
                false,
                false,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 2);

        if (useIntake) {
            groundIntake();
        } else {
            pushIntake();
        }

        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.AUTOGRAB);
        // Cycle
        for (int i = 1; i < 6; i++) {
            robot.drivetrain.goToPoint(
                    new Pose2d(ix, iy, -Math.PI / 2),
                    false,
                    true,
                    0.6
            );
            robot.ndeposit.startSpecimenIntake();
            robot.waitWhile(() -> robot.drivetrain.isBusy());

            robot.ndeposit.grab();
            robot.waitWhile(() -> robot.ndeposit.isGrabDone());

            robot.drivetrain.goToPoint(
                    new Pose2d(sx, sy, -Math.PI / 2),
                    false,
                    true,
                    1.0
            );

            // Extend out intake to prepare to search after depositing
            robot.nclawIntake.setSubmersibleIntake(true);
            robot.nclawIntake.setGrab(false);
            robot.nclawIntake.setExtendoTargetPos(5.0);
            robot.ndeposit.startSpecimenDeposit();
            robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.nclawIntake.isExtended());

            robot.drivetrain.goToPoint(
                    new Pose2d(dX[i], dY, -Math.PI / 2),
                    false,
                    true,
                    1.0
            );
            robot.waitWhile(() -> robot.drivetrain.isBusy());

            robot.ndeposit.deposit();

            robot.nclawIntake.setGrab(true);
            double startTime = System.currentTimeMillis();
            robot.waitWhile(() -> startTime - System.currentTimeMillis() < 750 && !robot.nclawIntake.hasSample());

            if(robot.nclawIntake.hasSample()) {
                robot.drivetrain.goToPoint(
                        new Pose2d(px, py, -Math.PI),
                        false,
                        true,
                        1.0
                );

                robot.waitWhile(() -> robot.nclawIntake.isTransferReady());
                robot.nclawIntake.finishTransfer();
                robot.ndeposit.finishTransfer();

                robot.waitWhile(() -> robot.drivetrain.isBusy() || robot.ndeposit.isTransferFinished());

                robot.ndeposit.outtake();
                robot.waitWhile(() -> robot.ndeposit.isOuttakeDone());
            }else{
                robot.drivetrain.goToPoint(
                        new Pose2d(mx, my, -Math.PI / 2),
                        false,
                        true,
                        1.0
                );
                robot.waitWhile(() -> robot.drivetrain.isBusy());
            }
        }
    }

    public void groundIntake() {
        // Get ground 1
        robot.drivetrain.goToPoint(
                new Pose2d(px, py, Math.atan2(b1y - py, b1x - px)),
                false,
                true,
                1.0
        );
        robot.nclawIntake.setGrab(false);
        robot.nclawIntake.setTargetPose(new Pose2d(b1x, b1y, -Math.PI / 2));
        robot.nclawIntake.extend();
        robot.waitWhile(() -> robot.drivetrain.isBusy() || !robot.nclawIntake.isExtended());

        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady());

        robot.ndeposit.outtake(); // this buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.waitWhile(() -> robot.ndeposit.isTransferFinished());

        robot.nclawIntake.setExtendoTargetPos(6.0);
        robot.nclawIntake.extend();
        robot.waitWhile(() -> !robot.ndeposit.isOuttakeDone());

        // Get ground 2
        robot.nclawIntake.setGrab(false);
        robot.drivetrain.goToPoint(
                new Pose2d(px, py, Math.atan2(b2y - py, b2x - px)),
                false,
                true,
                1.0
        );
        robot.nclawIntake.setTargetPose(new Pose2d(b2x, b2y, Math.PI / 2));
        robot.nclawIntake.extend();
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());

        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady());

        robot.ndeposit.outtake(); //this buffers it
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.waitWhile(() -> !robot.ndeposit.isTransferFinished());

        robot.nclawIntake.setExtendoTargetPos(6.0);
        robot.nclawIntake.extend();
        robot.waitWhile(() -> !robot.ndeposit.isOuttakeDone());

        // Get ground 3
        robot.nclawIntake.setGrab(false);
        robot.drivetrain.goToPoint(
                new Pose2d(px, py, Math.atan2(b3y - py, b3x - px)),
                false,
                true,
                1.0
        );
        robot.nclawIntake.setTargetPose(new Pose2d(b3x, b3y, Math.PI / 2));
        robot.nclawIntake.extend();
        robot.waitWhile(() -> !robot.nclawIntake.isExtended());

        robot.nclawIntake.setGrab(true);
        robot.waitWhile(() -> !robot.nclawIntake.isTransferReady());

        robot.ndeposit.outtake();
        robot.nclawIntake.finishTransfer();
        robot.ndeposit.finishTransfer();
        robot.waitWhile(() -> !robot.ndeposit.isTransferFinished());
    }

    public void pushIntake() {
        getBlockAt(b1x, b1y);
        getBlockAt(b2x, b2y);
        getBlockAt(b3x, b3y);
    }

    private double buffer = 2;

    private void getBlockAt(double x, double y) {
        robot.drivetrain.goToPoint(
                new Pose2d(x + Globals.ROBOT_WIDTH / 2 + buffer, y - Globals.ROBOT_REVERSE_LENGTH - buffer, -Math.PI / 2),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 1.0);

        robot.drivetrain.setBrakePad(true);
        robot.waitWhile(() -> (robot.sensors.getVelocity().toVec3().getMag() > 1.0));

        robot.drivetrain.setBrakePad(false);
        robot.drivetrain.goToPoint(
                new Pose2d(x, y - Globals.ROBOT_REVERSE_LENGTH - buffer, -Math.PI / 2),
                false,
                false,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.targetPoint.getDistanceFromPoint(robot.sensors.getOdometryPosition()) > 1.0);

        robot.drivetrain.goToPoint(
                new Pose2d(x + Globals.ROBOT_WIDTH - 2 * buffer, cy, -Math.PI / 2),
                false,
                true,
                1.0
        );
        robot.waitWhile(() -> robot.drivetrain.isBusy());
    }
}
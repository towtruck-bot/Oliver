package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTurret;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public static double slidesInc = 0.4;
    public static double extendoInc = 0.4;
    public static double verticalExtensionSpeed = 0.5;
    public static double horizontalExtensionSpeed = 0.35;
    public static double intakeClawRotationInc = 0.1;
    public static double intakeTurretRotationInc = -0.1;
    public static double extensionPreset = 15;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Globals.TESTING_DISABLE_CONTROL = false;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = false;

        Robot robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle lsb_1 = new ButtonToggle();
        ButtonToggle rsb_1 = new ButtonToggle();

        //Gamepad 2
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double triggerThresh = 0.2;
        final double triggerHardThresh = 0.7;
        double turretAngle = 0;
        double clawAngle = 0;
        boolean speciMode = false;
        boolean intakeMode = false;
        boolean high = true;
        boolean autoGrab = false;

        while (opModeInInit()) {
            robot.update();
            robot.ndeposit.presetDepositHeight(speciMode, high, false);
        }
        robot.nclawIntake.setTargetPose(new Pose2d(extensionPreset, 0, 0));
        robot.nclawIntake.setAutoEnableCamera(true);

        while (!isStopRequested()) {
            robot.update();

            if (autoGrab) {
                robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.AUTOGRAB);
                robot.nclawIntake.setTargetType(nClawIntake.Target.RELATIVE);
            } else {
                robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_AIM);
                robot.nclawIntake.setTargetType(nClawIntake.Target.RELATIVE);
            }
            robot.nclawIntake.setRetryGrab(false);

            if (x_1.isClicked(gamepad1.x) || b_2.isClicked(gamepad2.b)) {
                speciMode = !speciMode;
                intakeMode = false;
                robot.ndeposit.presetDepositHeight(speciMode, high, false);
                if (speciMode) gamepad1.rumble(250);
                else gamepad1.rumble(100);
            }
            if (b_1.isClicked(gamepad1.b)) {
                high = !high;
                robot.ndeposit.presetDepositHeight(speciMode, high, false);
            }
            if (y_1.isClicked(gamepad1.y)) {
                autoGrab = !autoGrab;
                if (autoGrab) gamepad1.rumble(250);
                else gamepad1.rumble(100);
            }
            if (lsb_1.isClicked(gamepad1.left_stick_button)) {
                intakeMode = !intakeMode && !speciMode;
                robot.ndeposit.holdSlides = false;
            }

            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (speciMode) {
                    // Begin specimen grab
                    if (robot.ndeposit.state == nDeposit.State.IDLE) robot.ndeposit.startSpecimenIntake();
                    // Finish specimen grab
                    else if (robot.ndeposit.state == nDeposit.State.SPECIMEN_INTAKE_WAIT) robot.ndeposit.grab();
                    else if (robot.ndeposit.state == nDeposit.State.HOLD) robot.ndeposit.startSpecimenDeposit();
                    else robot.ndeposit.deposit();
                } else {
                    if (robot.ndeposit.isDepositingSample()) robot.ndeposit.deposit();
                    // Begin sample intake
                    else if (robot.nclawIntake.state == nClawIntake.State.READY || robot.nclawIntake.state == nClawIntake.State.TRANSFER_WAIT) {
                        robot.nclawIntake.extend();
                        intakeMode = true;
                        robot.nclawIntake.target.heading = robot.nclawIntake.target.y = 0;
                        turretAngle = 0;
                        clawAngle = 0;
                    }
                    // Manual retract
                    else robot.nclawIntake.retract();
                }
            }

            if (robot.nclawIntake.state == nClawIntake.State.START_RETRACT) intakeMode = false;

            // Deposit sample/speci. The deposit FSM takes care of which one
            if (robot.nclawIntake.isOut()) {
                rb_1.isClicked(gamepad1.right_bumper);
                robot.nclawIntake.setGrab(gamepad1.right_bumper);

                robot.drivetrain.setBrakePad(robot.drivetrain.vdrive.mag() < 0.1 && Math.abs(robot.drivetrain.vturn) < 0.1);
            } else {
                robot.drivetrain.setBrakePad(false);
                if (rb_1.isClicked(gamepad1.right_bumper)) {
                    if (robot.ndeposit.isHolding()) {
                        robot.ndeposit.deposit();
                    } else {
                        robot.ndeposit.startSampleDeposit();
                        robot.nclawIntake.finishTransfer();
                        robot.ndeposit.finishTransfer();
                    }
                }
            }

            // Manualy adjust the slides height during deposit
            if (intakeMode) {
                double t = robot.nclawIntake.intakeTurret.getTargetTurretRotation();
                robot.nclawIntake.setTargetPose(new Pose2d(
                    Utils.minMaxClip(robot.nclawIntake.target.x - gamepad1.right_stick_y * verticalExtensionSpeed, 0, IntakeTurret.extendoOffset + IntakeExtension.maxExtendoLength + IntakeTurret.turretLengthTip * -Math.cos(t)),
                    Utils.minMaxClip(robot.nclawIntake.target.y - gamepad1.right_stick_x * horizontalExtensionSpeed, -IntakeTurret.turretLengthTip, IntakeTurret.turretLengthTip),
                    robot.nclawIntake.target.heading + intakeClawRotationInc * (gamepad1.left_trigger - gamepad1.right_trigger))
                );
                /*double turretControl1 = robot.drivetrain.smoothControls(gamepad1.right_stick_x);
                double extendoControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.nclawIntake.setExtendoTargetPos(robot.nclawIntake.getExtendoTargetPos() + extendoInc * extendoControl1);
                clawAngle += intakeClawRotationInc * (gamepad1.left_trigger - gamepad1.right_trigger);
                turretAngle += intakeTurretRotationInc * turretControl1;
                clawAngle = AngleUtil.mirroredClipAngle(clawAngle);
                turretAngle = Utils.minMaxClip(turretAngle, -1.7, 1.7);*/
            } else {
                double slidesControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() + slidesInc * slidesControl1);
            }

            robot.nclawIntake.setManualClawAngle(clawAngle - turretAngle);
            robot.nclawIntake.setManualTurretAngle(turretAngle);

            // Reset encoders in case something breaks
            if (rsb_1.isClicked(gamepad1.right_stick_button)) {
                if (gamepad1.left_trigger >= triggerHardThresh && gamepad1.right_trigger >= triggerHardThresh) {
                    robot.sensors.setOdometryPosition(0, 48, 0);
                    LogUtil.drivePositionReset = true;
                } else {
                    robot.sensors.hardwareResetSlidesEncoders();
                }
                gamepad1.rumble(250);
            }

            // Driving
            robot.drivetrain.intakeDriveMode = intakeMode;
            robot.drivetrain.drive(gamepad1, speciMode);

            // Toggle Alliance
            if (x_2.isClicked(gamepad2.x)) Globals.isRed = !Globals.isRed;

            // Increment/Decrement Intake Slides
            // Up --> increase, Down --> decrease on right joystick
            //double intakeControl2 = robot.drivetrain.smoothControls(-gamepad2.right_stick_y);
            //robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl2);

            // Increment/Decrement Deposit Slides
            // Up --> increase, Down --> decrease on left joystick
            //double slidesControl2 = robot.drivetrain.smoothControls(-gamepad2.left_stick_y);
            //robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc * slidesControl2);

            if (gamepad2.dpad_up) {
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() + slidesInc);
            } else if (gamepad2.dpad_down) {
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() - slidesInc);
            }

            // hang (both drivers)
            int hangLeftDir = 0, hangRightDir = 0;
            //for (Gamepad gamepad : new Gamepad[]{gamepad1, gamepad2}) {
            if (gamepad1.dpad_up) { ++hangLeftDir; ++hangRightDir; }
            if (gamepad1.dpad_down) { --hangLeftDir; --hangRightDir; }
            if (gamepad1.dpad_left) { --hangLeftDir; ++hangRightDir; }
            if (gamepad1.dpad_right) { ++hangLeftDir; --hangRightDir; }
            //}
            if (-gamepad2.right_stick_y >= triggerThresh) ++hangLeftDir;
            if (-gamepad2.right_stick_y <= -triggerThresh) --hangLeftDir;
            if (-gamepad2.left_stick_y >= triggerThresh) ++hangRightDir;
            if (-gamepad2.left_stick_y <= -triggerThresh) --hangRightDir;
            if (gamepad2.y) { ++hangLeftDir; ++hangRightDir; }
            if (gamepad2.a) { --hangLeftDir; --hangRightDir; }

            if (hangLeftDir > 0) robot.hang.leftUp();
            else if (hangLeftDir < 0) robot.hang.leftPull();
            else robot.hang.leftOff();
            if (hangRightDir > 0) robot.hang.rightUp();
            else if (hangRightDir < 0) robot.hang.rightPull();
            else robot.hang.rightOff();
            if (gamepad2.right_bumper) robot.hang.l3Pull();
            else if (gamepad2.left_bumper) robot.hang.l3Up();
            else robot.hang.l3Off();
            if (gamepad2.right_trigger >= triggerHardThresh) {
                robot.ndeposit.hangState = nDeposit.HangState.PULL;
            } else if (gamepad2.left_trigger >= triggerHardThresh || gamepad1.touchpad) {
                robot.ndeposit.hangState = nDeposit.HangState.OUT;
            }

            // Used to keep extendo in during hang
            if (gamepad1.back || gamepad2.back) robot.nclawIntake.intakeTurret.intakeExtension.forcePullIn();

            telemetry.addData("speciMode", speciMode);
            telemetry.addData("intakeMode", intakeMode);
            telemetry.addData("high", high);
            telemetry.addData("autoGrab", autoGrab);
            telemetry.addData("Intake target pos", robot.nclawIntake.getExtendoTargetPos());
            telemetry.addData("Deposit height", robot.ndeposit.getDepositHeight());
            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("Intake current length", robot.sensors.getExtendoPos());
            telemetry.addData("Slides current length", robot.sensors.getSlidesPos());
            telemetry.addData("claw angle", clawAngle);
            telemetry.addData("turret angle", turretAngle);
            telemetry.update();
        }
    }
}

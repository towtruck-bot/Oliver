package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.LogUtil;

@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
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
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle lsb_1 = new ButtonToggle();
        ButtonToggle rsb_1 = new ButtonToggle();

        //Gamepad 2
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double slidesInc = 0.4;
        final double extendoInc = 0.4;
        final double intakeClawRotationInc = 0.1;
        final double intakeTurretRotationInc = 0.1;
        final double triggerThresh = 0.2;
        boolean speciMode = false;
        boolean intakeMode = false;
        boolean high = true;

        while (opModeInInit()) {
            robot.update();
            robot.ndeposit.presetDepositHeight(speciMode, high);
        }

        while (!isStopRequested()) {
            robot.update();

            robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_AIM);
            robot.nclawIntake.setTargetType(nClawIntake.Target.MANUAL);

            // Toggle Specimen/Sample Deposit Mode
            if (x_1.isClicked(gamepad1.x)) {
                speciMode = !speciMode;
                robot.ndeposit.presetDepositHeight(speciMode, high);
            }
            if (b_2.isClicked(gamepad2.b)) {
                speciMode = !speciMode;
                intakeMode = false;
                robot.ndeposit.presetDepositHeight(speciMode, high);
            }
            // Toggle High/Low Deposit
            if (b_1.isClicked(gamepad1.b)) {
                high = !high;
                robot.ndeposit.presetDepositHeight(speciMode, high);
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
                    // Begin sample intake
                    if (robot.nclawIntake.state == nClawIntake.State.READY || robot.nclawIntake.state == nClawIntake.State.TRANSFER_WAIT) robot.nclawIntake.extend();
                    // Manual retract
                    else {
                        robot.nclawIntake.retract();
                        intakeMode = false;
                    }
                }
            }

            // Deposit sample/speci. The deposit FSM takes care of which one
            if (rb_1.isClicked(gamepad1.right_bumper)) {
                if (robot.nclawIntake.isExtended()) robot.nclawIntake.setGrab(true);
                else if (robot.nclawIntake.isTransferReady() && robot.ndeposit.isTransferReady()) {
                    robot.ndeposit.startSampleDeposit();
                    robot.nclawIntake.finishTransfer();
                    robot.ndeposit.finishTransfer();
                } else robot.ndeposit.deposit();
            }

            // Manualy adjust the slides height during deposit
            if (intakeMode) {
                double turretControl1 = robot.drivetrain.smoothControls(gamepad1.right_stick_x);
                double extendoControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.nclawIntake.setExtendoTargetPos(robot.nclawIntake.getExtendoTargetPos() + extendoInc * extendoControl1);
                robot.nclawIntake.setManualClawAngle(robot.nclawIntake.getManualClawAngle() + intakeClawRotationInc * (gamepad1.right_trigger - gamepad1.left_trigger));
                robot.nclawIntake.setManualTurretAngle(robot.nclawIntake.getManualTurretAngle() + intakeTurretRotationInc * turretControl1);
            } else {
                double slidesControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() + slidesInc * slidesControl1);
            }

            // Reset encoders in case something breaks
            if (rsb_1.isClicked(gamepad1.right_stick_button)) robot.sensors.resetSlidesEncoders();

            // Driving
            robot.drivetrain.intakeDriveMode = intakeMode;
            robot.drivetrain.drive(gamepad1);

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
            else if (hangLeftDir < 0) robot.hang.leftReverse();
            else robot.hang.leftOff();

            if (hangRightDir > 0) robot.hang.rightUp();
            else if (hangRightDir < 0) robot.hang.rightReverse();
            else robot.hang.rightOff();

            if (gamepad2.right_bumper) robot.hang.l3Pull();
            else if (gamepad2.left_bumper) robot.hang.l3Up();
            else robot.hang.l3Off();

            if (gamepad2.right_trigger >= 0.7) {
                robot.ndeposit.hangState = nDeposit.HangState.PULL;
            } else if (gamepad2.left_trigger >= 0.7 || gamepad1.touchpad) {
                robot.ndeposit.hangState = nDeposit.HangState.OUT;
            }

            // Used to keep intake extendo in during hang
            if (gamepad1.back || gamepad2.back) robot.nclawIntake.intakeTurret.intakeExtension.forcePullIn();

            telemetry.addData("speciMode", speciMode);
            telemetry.addData("intakeMode", intakeMode);
            telemetry.addData("high", high);
            telemetry.addData("Intake target pos", robot.nclawIntake.getExtendoTargetPos());
            telemetry.addData("Deposit height", robot.ndeposit.getDepositHeight());
            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("Intake current length", robot.sensors.getExtendoPos());
            telemetry.addData("Slides current length", robot.sensors.getSlidesPos());
            telemetry.update();
        }
    }
}

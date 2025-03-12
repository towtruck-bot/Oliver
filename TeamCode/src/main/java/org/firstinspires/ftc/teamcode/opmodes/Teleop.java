package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.LogUtil;

@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = false;

        Robot robot = new Robot(hardwareMap);
        robot.setAbortChecker(() -> !isStopRequested());
        LogUtil.init();

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle lt_1 = new ButtonToggle();
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
        final double intakeClawRotationInc = 0.1; // 0.08
        final double triggerThresh = 0.2;
        boolean speciMode = false;
        boolean high = true;

        while (opModeInInit()) {
            robot.update();
            robot.updateDepositHeights(speciMode, high);
        }

        while (!isStopRequested()) {
            robot.update();
            Robot.RobotState robotState = robot.getState();

            // DRIVER 1

            // Toggle Specimen/Sample Deposit Mode
            if (x_1.isClicked(gamepad1.x)) {
                speciMode = !speciMode;
                robot.updateDepositHeights(speciMode, high);
            }
//liam is so cool lol
            // Toggle High/Low Deposit
            if (b_1.isClicked(gamepad1.b)) {
                high = !high;
                robot.updateDepositHeights(speciMode, high);
            }

            // Increment / Decrement Slides Height
            if (gamepad1.y) {
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc);
            } else if (gamepad1.a) {
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() - slidesInc);
            }

//LIAM IS THE BEST EVER!
            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (robotState == Robot.RobotState.IDLE) {
                    robot.setNextState(speciMode ? Robot.NextState.GRAB_SPECIMEN : Robot.NextState.INTAKE_SAMPLE);
                } else if (robotState == Robot.RobotState.SAMPLE_READY) {
                    robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
                } else if (robotState == Robot.RobotState.GRAB_SPECIMEN && speciMode || robotState == Robot.RobotState.SPECIMEN_READY) {
                    robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
                } else {
                    robot.setNextState(Robot.NextState.DONE);
                }
            }

            if (robot.clawIntake.isExtended()) {
                rb_1.isClicked(gamepad1.right_bumper);
                lt_1.isClicked(gamepad1.left_trigger > triggerThresh);
                robot.clawIntake.grab(gamepad1.right_bumper);
                robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc * (gamepad1.left_trigger - gamepad1.right_trigger));
            } else {
                if (rb_1.isClicked(gamepad1.right_bumper))
                    robot.setNextState(Robot.NextState.DEPOSIT);

                // neil says he never uses this during teleop, and its not in the control diagram he gave us??? not sure why this is here
                if (lt_1.isClicked(gamepad1.left_trigger > triggerThresh) && robot.clawIntake.isRetracted()) robot.setNextState(Robot.NextState.DONE);
            }

            if (robotState == Robot.RobotState.DEPOSIT_BUCKET || robotState == Robot.RobotState.DEPOSIT_SPECIMEN) {
                double slidesControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc * slidesControl1);
            } else if (robotState == Robot.RobotState.INTAKE_SAMPLE) {
                double intakeControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl1);
            }

            if (rsb_1.isClicked(gamepad1.right_stick_button)) {
                robot.deposit.slides.resetSlidesEncoders();
                robot.clawIntake.resetExtendoEncoders();
            }

            if (lsb_1.isClicked(gamepad1.left_stick_button)) {
                robot.restartState();
            }

            // Driving
            robot.drivetrain.slow = robot.clawIntake.isExtended();
            robot.drivetrain.drive(gamepad1);

            // Driver 2

            // Toggle Alliance
            if (x_2.isClicked(gamepad2.x)) {
                Globals.isRed = !Globals.isRed;
            }

            // Specimen/Sample Toggle
            if (b_2.isClicked(gamepad2.b)) {
                speciMode = !speciMode;
                robot.updateDepositHeights(speciMode, high);
            }

            // Increment/Decrement Intake Slides
            // Up --> increase, Down --> decrease on right joystick
            //double intakeControl2 = robot.drivetrain.smoothControls(-gamepad2.right_stick_y);
            //robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl2);

            // Increment/Decrement Deposit Slides
            // Up --> increase, Down --> decrease on left joystick
            //double slidesControl2 = robot.drivetrain.smoothControls(-gamepad2.left_stick_y);
            //robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc * slidesControl2);

            // hang (both drivers)
            int hangLeftDir = 0, hangRightDir = 0;
            for (Gamepad gamepad : new Gamepad[]{gamepad1, gamepad2}) {
                if (gamepad.dpad_up) { ++hangLeftDir; ++hangRightDir; }
                if (gamepad.dpad_down) { --hangLeftDir; --hangRightDir; }
                if (gamepad.dpad_left) { --hangLeftDir; ++hangRightDir; }
                if (gamepad.dpad_right) { ++hangLeftDir; --hangRightDir; }
            }
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
                robot.deposit.hangMode = Deposit.HangMode.PULL;
            } else if (gamepad2.left_trigger >= 0.7) {
                robot.deposit.hangMode = Deposit.HangMode.OUT;
            }

            telemetry.addData("speciMode", speciMode);
            telemetry.addData("high", high);
            telemetry.addData("Intake target pos", robot.clawIntake.getIntakeTargetPos());
            telemetry.addData("Intake claw rotation", robot.clawIntake.getClawRotAngle());
            telemetry.addData("deposit height", robot.deposit.getDepositHeight());
            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("robotState", robotState);
            telemetry.addData("Intake current length", robot.sensors.getExtendoPos());
            telemetry.addData("Slides current length", robot.deposit.slides.getLength());
            telemetry.addData("hasSamplePreload", Globals.hasSamplePreload);
            telemetry.update();
        }
    }
}

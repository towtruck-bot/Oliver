package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class nTeleop extends LinearOpMode {
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;

        Robot robot = new Robot(hardwareMap);

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle rsb_1 = new ButtonToggle();

        //Gamepad 2
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double slidesInc = 0.4;
        final double extendoInc = 0.4;
        final double intakeClawRotationInc = 0.1; // 0.08
        boolean speciMode = false;
        boolean high = true;

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            robot.update();

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

            // Transition Between Intake or Deposit FSMs through Robot
            // rb(held) --> lower claw and keep open, rb(released) --> raise claw and keep open
            // lb --> close claw and grab, then retract and return to transfer position

            if (robot.clawIntake.isExtended()) {
                robot.clawIntake.grab(gamepad1.right_bumper);
            } else if (robot.clawIntake.isRetracted()) {
                if (rb_1.isClicked(gamepad1.right_bumper)) {
                    robot.setNextState(Robot.NextState.DEPOSIT);
                }
            }
//LIAM IS THE BEST EVER!
            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (robot.getState() == Robot.RobotState.IDLE) {
                    robot.setNextState(speciMode ? Robot.NextState.GRAB_SPECIMEN : Robot.NextState.INTAKE_SAMPLE);
                } else {
                    robot.setNextState(Robot.NextState.DONE);
                }
            }

            if (robot.getState() == Robot.RobotState.DEPOSIT_BUCKET) {
                double slidesControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc * slidesControl1);
            } else {
                double intakeControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl1);
            }
            robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc * (gamepad1.right_trigger - gamepad1.left_trigger));

            if (rsb_1.isClicked(gamepad1.right_stick_button)) {
                robot.deposit.slides.resetSlidesEncoders();
                robot.clawIntake.resetExtendoEncoders();
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
            if (a_2.isClicked(gamepad2.a)) {
                speciMode = !speciMode;
                robot.updateDepositHeights(speciMode, high);
            }

            // Force Deposit Slides Retract
            if (b_2.isClicked(gamepad2.b)) {
                robot.deposit.setDepositHeight(0.0);
            }

            // Increment/Decrement Intake Slides
            // Up --> increase, Down --> decrease on right joystick
            double intakeControl2 = robot.drivetrain.smoothControls(-gamepad2.right_stick_y);
            robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl2);

            // Increment/Decrement Deposit Slides
            // Up --> increase, Down --> decrease on left joystick
            double slidesControl2 = robot.drivetrain.smoothControls(-gamepad2.left_stick_y);
            robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc * slidesControl2);

            telemetry.addData("speciMode", speciMode);
            telemetry.addData("high", high);
            telemetry.addData("intake target pos", robot.clawIntake.getIntakeTargetPos());
            telemetry.addData("intake claw rotation", robot.clawIntake.getClawRotAngle());
            telemetry.addData("deposit height", robot.deposit.getDepositHeight());
            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("robot state", robot.getState());
            telemetry.addData("Slides: Length", robot.deposit.slides.getLength());
            telemetry.addData("hasSamplePreload", Globals.hasSamplePreload);
            telemetry.update();
        }
    }
}

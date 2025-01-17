package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class nTeleop extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle a_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();

        //Gamepad 2
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle y_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double triggerThresh = 0.2;
        final double slidesInc = 0.5;
        final double extendoInc = 0.5;
        final double intakeClawRotationInc = 1.0;
        boolean speciMode = true;
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

            // Toggle High/Low Deposit
            if (b_1.isClicked(gamepad1.b)) {
                high = !high;
                robot.updateDepositHeights(speciMode, high);
            }

            // Increment / Decrement Slides Height
            if (y_1.isHeld(gamepad1.y, 5)) {
                robot.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc);
            } else if (a_1.isHeld(gamepad1.a, 5)) {
                robot.setDepositHeight(robot.deposit.getDepositHeight() - slidesInc);
            }

            // Transition Between Intake or Deposit FSMs through Robot
            // lb --> Deposit sample/specimen(i.e. let go of) AND intake grab/retract
            // rb(held) --> lower claw and grab motion, rb(unheld) --> claw returns to wait-grab angle
            if (rb_1.isHeld(gamepad1.right_bumper, 50) && !speciMode) {
                robot.grabSample();

                if (lb_1.isClicked(gamepad1.left_bumper)) {
                    robot.retractIntake();
                }
            } else {
                if (rb_1.isReleased(gamepad1.right_bumper)) {
                    robot.setNextState(Robot.NextState.DONE);
                }
            }

            if (rb_1.isClicked(gamepad1.right_bumper) && speciMode) {
                robot.setNextState(Robot.NextState.DEPOSIT);//TODO: check this
            }

            if (lb_1.isClicked(gamepad1.left_bumper) && robot.getState() != Robot.RobotState.INTAKE_SAMPLE) {
                robot.setNextState(Robot.NextState.DEPOSIT);
            }
//
//            if (lb_1.isClicked(gamepad1.left_bumper)) {
//                if (robot.getState() == Robot.RobotState.IDLE) {
//                    robot.setNextState(speciMode ? Robot.NextState.GRAB_SPECIMEN : Robot.NextState.INTAKE_SAMPLE);
//                } else {
//                    robot.setNextState(Robot.NextState.DONE);
//                }
//            } else if (rb_1.isClicked(gamepad1.right_bumper)) {
//                robot.setNextState(Robot.NextState.DEPOSIT);
//            }

            // Rotate Intake Claw + Grab Specimen
            // lt --> rotate left, rt --> rotate right(if in not specimen mode), rt --> grab specimen(if in specimen mode)
            // TODO: Check if left/right is plus/minus or vice versa
            if (gamepad1.left_trigger > triggerThresh) {
                robot.setIntakeClawAngle(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc);
            } else if (gamepad1.right_trigger > triggerThresh) {
                if (speciMode) {
                    robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
                } else {
                    robot.setIntakeClawAngle(robot.clawIntake.getClawRotAngle() - intakeClawRotationInc);
                }
            }

            // Intake extension
            // Right joystick down --> retract, right joystick up --> extend
            double intakeControl1 = robot.drivetrain.smoothControls(gamepad1.right_stick_y);
            if (Math.abs(intakeControl1) > 0.2) {
                robot.setIntakeLength(robot.clawIntake.getExtendoPos() + extendoInc * Math.signum(intakeControl1));
            }

            // Driving
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
                robot.setDepositHeight(0.0);
            }

            // Increment/Decrement Intake Slides
            // Up --> increase, Down --> decrease on right joystick
            double intakeControl2 = robot.drivetrain.smoothControls(gamepad2.right_stick_y);
            if (Math.abs(intakeControl2) > 0.2) {
                robot.setIntakeLength(robot.clawIntake.getExtendoPos() + extendoInc * Math.signum(intakeControl2));
            }

            // Increment/Decrement Deposit Slides
            // Up --> increase, Down --> decrease on left joystick
            double slidesControl2 = robot.drivetrain.smoothControls(gamepad2.left_stick_y);
            if (Math.abs(slidesControl2) > 0.2) {
                robot.setDepositHeight(robot.deposit.getDepositHeight() + Math.signum(slidesControl2) * slidesInc);
            }

            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("speciMode", speciMode);
            telemetry.addData("robot state", robot.getState());
            telemetry.addData("deposit height", robot.deposit.getDepositHeight());
            telemetry.update();
        }
    }
}

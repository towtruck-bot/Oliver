package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "Test")
public class RobotTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);

        final double extendoInc = 0.4;
        final double intakeClawRotationInc = 0.08;
        final double slidesInc = 0.4;

        ButtonToggle button_a = new ButtonToggle();
        ButtonToggle button_b = new ButtonToggle();
        ButtonToggle button_y = new ButtonToggle();
        ButtonToggle button_x = new ButtonToggle();
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();

        waitForStart();

        while (opModeIsActive()) {
            if (button_a.isClicked(gamepad1.a))
                robot.setNextState(Robot.NextState.DEPOSIT);
            if (button_b.isClicked(gamepad1.b))
                robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
            if (button_y.isClicked(gamepad1.y))
                robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
            if (button_x.isClicked(gamepad1.x))
                robot.setNextState(Robot.NextState.DONE);

            if (robot.clawIntake.isExtended()) {
                robot.clawIntake.grab(gamepad1.right_bumper);
            } else if (robot.clawIntake.isRetracted()) {
                if (rb_1.isClicked(gamepad1.right_bumper)) {
                    robot.setNextState(Robot.NextState.DEPOSIT);
                }
            }

            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (robot.getState() == Robot.RobotState.IDLE) {
                    robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
                } else {
                    robot.setNextState(Robot.NextState.DONE);
                }
            }

            if (gamepad1.dpad_left)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc);
            if (gamepad1.dpad_right)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() - extendoInc);
            if (gamepad1.dpad_up)
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesInc);
            if (gamepad1.dpad_down)
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() - slidesInc);

            double intakeControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
            robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc * intakeControl1);
            robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc * (gamepad1.right_trigger - gamepad1.left_trigger));

            robot.update();

            telemetry.addData("robot state", robot.getState());
            telemetry.addData("intake target pos", robot.clawIntake.getIntakeTargetPos());
            telemetry.addData("intake claw rotation", robot.clawIntake.getClawRotAngle());
            telemetry.addData("deposit height", robot.deposit.getDepositHeight());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class ClawIntakeTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);

        final double triggerThresh = 0.2;
        final double extendoInc = 0.3;
        final double intakeClawRotationInc = 0.05;

        ButtonToggle button_a = new ButtonToggle();
        ButtonToggle button_b = new ButtonToggle();
        ButtonToggle button_y = new ButtonToggle();
        ButtonToggle button_x = new ButtonToggle();

        waitForStart();

        while (opModeIsActive()) {
            if (button_a.isClicked(gamepad1.y)) robot.clawIntake.extend();
            if (button_b.isClicked(gamepad1.b)) robot.clawIntake.grab(!robot.clawIntake.hasSample());
            if (button_y.isClicked(gamepad1.a)) robot.clawIntake.retract();
            if (button_x.isClicked(gamepad1.x)) robot.clawIntake.release();

            if (gamepad1.dpad_left)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc);
            if (gamepad1.dpad_right)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() - extendoInc);

            robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc * (gamepad1.right_trigger - gamepad1.left_trigger));

            robot.update();

            telemetry.addData("intake target pos", robot.clawIntake.getIntakeTargetPos());
            telemetry.addData("intake claw rotation", robot.clawIntake.getClawRotAngle());
            telemetry.update();
        }
    }
}

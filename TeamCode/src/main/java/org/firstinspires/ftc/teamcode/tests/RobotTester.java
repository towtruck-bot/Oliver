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
public class RobotTester extends LinearOpMode {

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
            if (button_a.isClicked(gamepad1.a)) robot.setNextState(Robot.NextState.DEPOSIT);
            if (button_b.isClicked(gamepad1.b)) robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
            if (button_y.isClicked(gamepad1.y)) robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
            if (button_x.isClicked(gamepad1.x)) robot.setNextState(Robot.NextState.DONE);

            if (robot.clawIntake.isExtended()) robot.clawIntake.grab(gamepad1.right_bumper);

            if (gamepad1.dpad_up)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() + extendoInc);
            if (gamepad1.dpad_down)
                robot.clawIntake.setIntakeTargetPos(robot.clawIntake.getIntakeTargetPos() - extendoInc);

            if (gamepad1.left_trigger > triggerThresh) robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() + intakeClawRotationInc);
            if (gamepad1.right_trigger > triggerThresh) robot.clawIntake.setClawRotation(robot.clawIntake.getClawRotAngle() - intakeClawRotationInc);

            robot.update();

            telemetry.addData("robot state", robot.getState());
            telemetry.addData("intake target pos", robot.clawIntake.getIntakeTargetPos());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;

        Robot robot = new Robot(hardwareMap);

        final double triggerThreshold = 0.2;

        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }

/*
LT finish deposit; outtake; cancel intake
RT grab specimen
LB deposit sample/specimen
RB intake sample
A unjam intake
Y on/reverse intake
B outtake > idle
X outtake > grab specimen
*/

        while (!isStopRequested()) {
            if (gamepad1.left_trigger > triggerThreshold) robot.setNextState(Robot.NextState.DONE);
            else if (gamepad1.left_bumper) robot.setNextState(Robot.NextState.DEPOSIT);
            else if (gamepad1.right_bumper) robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
            else if (gamepad1.right_trigger > triggerThreshold) robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
            if (gamepad1.b) robot.setOuttakeAndThenGrab(false);
            else if (gamepad1.x) robot.setOuttakeAndThenGrab(true);

            if (gamepad1.y) {
                if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.ON) robot.intake.setRollerReverse();
                else robot.intake.setRollerOn();
            } else if (gamepad1.a) robot.intake.setRollerUnjam();

            robot.drivetrain.drive(gamepad1);

            robot.update();

            telemetry.update();
        }
    }
}

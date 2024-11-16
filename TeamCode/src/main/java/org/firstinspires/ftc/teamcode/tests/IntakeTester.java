package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class IntakeTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);

        final double triggerThreshold = 0.2;
        final double intakeAdjustmentSpeed = 0.3;
        boolean didToggleDisable = false;
        boolean didToggleIntakeRoller = false;

        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }

/*
RT enable/disable
LT transfer
LB retract
RB extend
A unjam intake
Y on/reverse intake
< intake further out
> intake further in
*/

        while (!isStopRequested()) {
/*
            if (gamepad1.right_trigger > triggerThreshold) {
                if (!didToggleDisable) {
                    Globals.TESTING_DISABLE_CONTROL = !Globals.TESTING_DISABLE_CONTROL;
                    didToggleDisable = true;
                }
            } else {
                didToggleDisable = false;
            }

            if (gamepad1.left_trigger > triggerThreshold) robot.intake.transfer();
            else if (gamepad1.left_bumper) robot.intake.retract();
            else if (gamepad1.right_bumper) robot.intake.extend();

            if (gamepad1.y) {
                if (!didToggleIntakeRoller) {
                    if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.ON) robot.intake.setRollerReverse();
                    else robot.intake.setRollerOn();
                    didToggleIntakeRoller = true;
                }
            } else {
                didToggleIntakeRoller = false;
            }
            if (gamepad1.a) robot.intake.setRollerUnjam();
            if (gamepad1.dpad_left) robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed);
            else if (gamepad1.dpad_right) robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() - intakeAdjustmentSpeed);
*/
            robot.update();

            telemetry.addData("Globals.TESTING_DISABLE_CONTROL", Globals.TESTING_DISABLE_CONTROL);
//            telemetry.addData("Intake.intakeState", robot.intake.getIntakeState().toString());
//            telemetry.addData("Intake.targetPositionWhenExtended", robot.intake.getTargetPositionWhenExtended());
            telemetry.addData("Extendo position", robot.sensors.getIntakeExtensionPosition());
            telemetry.addData("Intake color", robot.sensors.getIntakeColor().toString());

            telemetry.update();
        }
    }
}

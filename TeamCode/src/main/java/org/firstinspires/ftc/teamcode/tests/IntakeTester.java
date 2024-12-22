package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp(group = "Test")
public class IntakeTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = false;

        Robot robot = new Robot(hardwareMap);

        final double triggerThreshold = 0.2;
        final double intakeAdjustmentSpeed = 0.3;
        final double intakeHeightStep = 5;
        boolean didToggleDisable = false;
        boolean didToggleAlliance = false;
        boolean didToggleIntakeRoller = false;
        boolean didToggleIntakeHeight = false;

        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }

/*
P1 ==== A
LT retract
LB transfer
RB extend
X roller off
A roller unjam
B roller on/reverse
< roller slow reverse
> roller keep in
^ intake further out
v intake further in
P2 ==== B
RT enable/disable
RSY intake slides
X red/blue
A intake down
Y intake up
*/

        while (!isStopRequested()) {
            if (gamepad2.right_trigger > triggerThreshold) {
                if (!didToggleDisable) {
                    Globals.TESTING_DISABLE_CONTROL = !Globals.TESTING_DISABLE_CONTROL;
                    didToggleDisable = true;
                }
            } else {
                didToggleDisable = false;
            }

            if (gamepad2.x) {
                if (!didToggleAlliance) {
                    Globals.isRed = !Globals.isRed;
                    didToggleAlliance = true;
                }
            } else {
                didToggleAlliance = false;
            }

            if (gamepad1.left_trigger > triggerThreshold) robot.intake.retract();
            else if (gamepad1.left_bumper) robot.intake.transfer();
            else if (gamepad1.right_bumper) robot.intake.extend();

            if (gamepad1.b) {
                if (!didToggleIntakeRoller) {
                    if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.ON) robot.intake.setRollerReverse();
                    else robot.intake.setRollerOn();
                    didToggleIntakeRoller = true;
                }
            } else {
                didToggleIntakeRoller = false;
            }
            if (gamepad1.x) robot.intake.setRollerOff();
            else if (gamepad1.a) robot.intake.setRollerUnjam();
            else if (gamepad1.dpad_left) robot.intake.setRollerSlowReverse();
            else if (gamepad1.dpad_right) robot.intake.setRollerKeepIn();

            if (gamepad1.dpad_up) robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed);
            else if (gamepad1.dpad_down) robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() - intakeAdjustmentSpeed);
            robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed * -gamepad2.right_stick_y);

            if (gamepad2.a) {
                if (!didToggleIntakeHeight) {
                    robot.intake.setFlipDownAngle(robot.intake.getFlipDownAngle() + intakeHeightStep);
                    didToggleIntakeHeight = true;
                }
            } else if (gamepad2.y) {
                if (!didToggleIntakeHeight) {
                    robot.intake.setFlipDownAngle(robot.intake.getFlipDownAngle() - intakeHeightStep);
                    didToggleIntakeHeight = true;
                }
            } else {
                didToggleIntakeHeight = false;
            }

            robot.drivetrain.drive(gamepad1);

            robot.update();

            telemetry.addData("Globals.TESTING_DISABLE_CONTROL", Globals.TESTING_DISABLE_CONTROL);
            telemetry.addData("Globals.isRed", Globals.isRed);
            telemetry.addData("Intake.intakeState", robot.intake.getIntakeState().toString());
            telemetry.addData("Intake.intakeRollerState", robot.intake.getIntakeRollerState().toString());
            telemetry.addData("Intake.targetPositionWhenExtended", robot.intake.getTargetPositionWhenExtended());
            telemetry.addData("Intake.flipDownAngle", robot.intake.getFlipDownAngle());
            telemetry.addData("Extendo position", robot.sensors.getIntakeExtensionPosition());
            telemetry.addData("Intake color", robot.sensors.getIntakeColor().toString());

            telemetry.update();
        }
    }
}


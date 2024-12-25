package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Disabled
@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;

        Robot robot = new Robot(hardwareMap);

        final double triggerThreshold = 0.2;
        final double intakeAdjustmentSpeed = 0.3;
        final double intakeHeightStep = 5;
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
LT finish deposit; start outtake; cancel intake
RT start grab specimen
LB close claw; start deposit sample/specimen
RB start intake sample
X roller off
A unjam intake
B on/reverse intake
< roller slow reverse
> roller keep in
^ intake further out
v intake further in
P2 ==== B
RSY intake slides
X red/blue
A intake down
Y intake up
*/

        while (!isStopRequested()) {
            if (gamepad2.x) {
                if (!didToggleAlliance) {
                    Globals.isRed = !Globals.isRed;
                    didToggleAlliance = true;
                }
            } else {
                didToggleAlliance = false;
            }

            if (gamepad1.left_trigger > triggerThreshold) robot.setNextState(Robot.NextState.DONE);
            else if (gamepad1.left_bumper) robot.setNextState(Robot.NextState.DEPOSIT);
            else if (gamepad1.right_bumper) robot.setNextState(Robot.NextState.INTAKE_SAMPLE);
            else if (gamepad1.right_trigger > triggerThreshold) robot.setNextState(Robot.NextState.GRAB_SPECIMEN);
            //if (gamepad1.y) robot.setOuttakeAndThenGrab(false);
            //else if (gamepad1.x) robot.setOuttakeAndThenGrab(true);

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

            if(gamepad2.left_bumper){
                robot.intake.intakeState = Intake.IntakeState.RETRACTING;
            }

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

            telemetry.addData("Globals.isRed", Globals.isRed);
            telemetry.addData("Intake.intakeState", robot.intake.getIntakeState().toString());
            telemetry.addData("Intake.intakeRollerState", robot.intake.getIntakeRollerState().toString());
            telemetry.addData("Intake.targetPositionWhenExtended", robot.intake.getTargetPositionWhenExtended());
            telemetry.addData("Intake.flipDownAngle", robot.intake.getFlipDownAngle());

            telemetry.update();
        }
    }
}

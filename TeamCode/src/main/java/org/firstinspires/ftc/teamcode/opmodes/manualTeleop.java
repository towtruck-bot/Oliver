package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class manualTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Globals.TESTING_DISABLE_CONTROL = false;

        Robot robot = new Robot(hardwareMap);
        ButtonToggle rightBumper = new ButtonToggle();

        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            robot.update();
        }

        boolean clawOpen = true;
        double armPos = 0;
        double wristPos = 0.5;
        final double intakeAdjustmentSpeed = 0.3;
        boolean didToggleIntakeRoller = false;
        boolean didToggleAlliance = false;

        robot.deposit.arm.setClawSpeciPrepare();

/*
Driver A
x -> wrist rotation up
b -> wrist rotation down
y -> arm rotation up
a -> arm rotation down
rb -> toggle claw grip
rt -> trigger intake
dpad_left -> rollers slow reversed
dpad_right -> roller keep curr sample in
dpad_up -> intake extendo out more by increment
dpad_down -> intake extendo in more by increment

Driver B
a -> roller unjam
b -> roller toggle direction
x -> intake roller off
y -> retract override
dpad_up -> alliance toggle(default blue)
right stick -> move intake back and forth
 */



        while (!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);

            //x -> claw grip close
            if(rightBumper.isClicked(gamepad1.right_bumper)){
                if (clawOpen) {
                    robot.deposit.arm.setClawSpeciGrab();
                }
                else {
                    robot.deposit.arm.setClawSpeciPrepare();
                }
                clawOpen = !clawOpen;
            }

            if(gamepad1.y){
                armPos-=0.005;
            }

            if(gamepad1.a){
                armPos+=0.005;
            }

            robot.deposit.arm.armRotation.setTargetPos(armPos);

            if (gamepad1.x) {
                wristPos += 0.005;
                robot.deposit.arm.clawRotation.setTargetPos(wristPos);
            }
            if (gamepad1.b) {
                wristPos -= 0.005;
                robot.deposit.arm.clawRotation.setTargetPos(wristPos);
            }
            Log.i("james", String.valueOf(armPos));
            Log.i("james", String.valueOf(wristPos));

            TelemetryUtil.packet.put("armPos: ", armPos);
            TelemetryUtil.packet.put("wristPos: ", wristPos);

            if (armPos <= 0) {
                armPos = 0;
            }
            else if (armPos > 1) {
                armPos = 1;
            }

            if (wristPos < 0) {
                wristPos = 0;
            }
            else if (wristPos > 1) {
                wristPos = 1;
            }

            if(gamepad1.right_trigger > 0.2){
                robot.intake.extend();
            }

            if(gamepad2.y){
                robot.intake.retract();
            }

            if (gamepad1.dpad_up)
                robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed);
            else if (gamepad1.dpad_down)
                robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() - intakeAdjustmentSpeed);


            if (gamepad2.b) {
                if (!didToggleIntakeRoller) {
                    if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.ON) robot.intake.setRollerReverse();
                    else robot.intake.setRollerOn();
                    didToggleIntakeRoller = true;
                }
            } else {
                didToggleIntakeRoller = false;
            }

            if (gamepad2.dpad_up) {
                if (!didToggleAlliance) {
                    Globals.isRed = !Globals.isRed;
                    didToggleAlliance = true;
                }
            } else {
                didToggleAlliance = false;
            }

            if (gamepad2.x)
                robot.intake.setRollerOff();
            else if (gamepad2.a)
                robot.intake.setRollerUnjam();
            else if (gamepad1.dpad_left)
                robot.intake.setRollerSlowReverse();
            else if (gamepad1.dpad_right)
                robot.intake.setRollerKeepIn();

            robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed * -gamepad2.right_stick_y);

            robot.update();

            telemetry.update();
        }

    }
}

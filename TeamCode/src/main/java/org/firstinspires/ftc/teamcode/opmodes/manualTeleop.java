//package org.firstinspires.ftc.teamcode.opmodes;
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
//import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
//import org.firstinspires.ftc.teamcode.utils.Globals;
//import org.firstinspires.ftc.teamcode.utils.RunMode;
//import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
//
//@TeleOp
//public class manualTeleop extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Globals.RUNMODE = RunMode.TELEOP;
//        Globals.TESTING_DISABLE_CONTROL = false;
//
//        Robot robot = new Robot(hardwareMap);
//        ButtonToggle rightBumper = new ButtonToggle();
//
//        telemetry.addData("State", "READY TO START");
//        telemetry.update();
//
//        while (opModeInInit()) {
//            robot.update();
//        }
//
//        boolean clawOpen = true;
//        double armPos = 0.045;
//        double wristPos = 0.285;
//        final double intakeAdjustmentSpeed = 0.3;
//        boolean didToggleIntakeRoller = false;
//        boolean didToggleAlliance = false;
//        boolean didToggleIntakeHeight = false;
//        boolean didToggleIntake = false;
//        final double intakeHeightStep = 5;
//
//        robot.deposit.arm.setClawSpeciPrepare();
//
///*
//Driver A
//x -> wrist rotation up
//b -> wrist rotation down
//y -> arm rotation up
//a -> arm rotation down
//rb -> toggle claw grip
//rt -> toggle intake
//lt -> start/stop intake
//lb -> reverse/on intake
//dpad_left -> actuation up increment
//dpad_right -> actuation down increment
//dpad_up -> intake extendo out more
//dpad_down -> intake extendo in more
//left stick -> grab
//right stick -> deposit 1
//touchpad -> deposit 2
//Driver B
//a -> roller unjam
//b -> roller on/off
//x -> roller reverse/on
//y -> intake retract
//dpad_up -> alliance toggle (default blue)
//dpad_right -> extendo position reset
//right stick -> move intake back and forth
// */
//
//        while (!isStopRequested()) {
//            robot.drivetrain.drive(gamepad1);
//
//            //x -> claw grip close
//            if(rightBumper.isClicked(gamepad1.right_bumper)){
//                if (clawOpen) {
//                    robot.deposit.arm.setClawSpeciGrab();
//                }
//                else {
//                    robot.deposit.arm.setClawSpeciPrepare();
//                }
//                clawOpen = !clawOpen;
//            }
//
//            if(gamepad1.y){
//                armPos-=0.005;
//            }
//            if(gamepad1.a){
//                armPos+=0.005;
//            }
//
//            if (gamepad1.x) {
//                wristPos += 0.005;
//            }
//            if (gamepad1.b) {
//                wristPos -= 0.005;
//            }
//
//            if (gamepad1.left_stick_button) {
//                // grab
//                armPos = 0.65;
//                wristPos = 0.315;
//            }
//            if (gamepad1.right_stick_button) {
//                // deposit 1
//                armPos = 0.435;
//                wristPos = 0.0;
//            }
//            if (gamepad1.touchpad) {
//                // deposit 2
//                armPos = 0.515;
//                wristPos = 0.0;
//            }
//
//            robot.deposit.arm.armRotation.setTargetPos(armPos);
//            robot.deposit.arm.clawRotation.setTargetPos(wristPos);
//
//            if (gamepad1.dpad_left) {
//                if (!didToggleIntakeHeight) {
//                    // robot.intake.setFlipDownAngle(robot.intake.getFlipDownAngle() + intakeHeightStep);
//                    didToggleIntakeHeight = true;
//                }
//            } else if (gamepad1.dpad_right) {
//                if (!didToggleIntakeHeight) {
//                    // robot.intake.setFlipDownAngle(robot.intake.getFlipDownAngle() - intakeHeightStep);
//                    didToggleIntakeHeight = true;
//                }
//            } else {
//                didToggleIntakeHeight = false;
//            }
//
//            Log.i("james", String.valueOf(armPos));
//            Log.i("james", String.valueOf(wristPos));
//
//            TelemetryUtil.packet.put("armPos: ", armPos);
//            TelemetryUtil.packet.put("wristPos: ", wristPos);
//
//            if (armPos <= 0) {
//                armPos = 0;
//            }
//            else if (armPos > 1) {
//                armPos = 1;
//            }
//
//            if (wristPos < 0) {
//                wristPos = 0;
//            }
//            else if (wristPos > 1) {
//                wristPos = 1;
//            }
//
//            if (gamepad1.dpad_up)
//                robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed);
//            else if (gamepad1.dpad_down)
//                robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() - intakeAdjustmentSpeed);
//            robot.intake.setTargetPositionWhenExtended(robot.intake.getTargetPositionWhenExtended() + intakeAdjustmentSpeed * -gamepad2.right_stick_y);
//
//
//            if (gamepad1.left_trigger > 0.2 || gamepad2.b) {
//                if (!didToggleIntakeRoller) {
//                    if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.ON) robot.intake.setRollerOff();
//                    else robot.intake.setRollerOn();
//                    didToggleIntakeRoller = true;
//                }
//            } else if (gamepad2.x || gamepad1.left_bumper) {
//                if (!didToggleIntakeRoller) {
//                    if (robot.intake.getIntakeRollerState() == Intake.IntakeRollerState.REVERSE) robot.intake.setRollerOn();
//                    else robot.intake.setRollerReverse();
//                    didToggleIntakeRoller = true;
//                }
//            } else {
//                didToggleIntakeRoller = false;
//            }
//            if (gamepad2.a) robot.intake.setRollerUnjam();
//            //else if (gamepad1.dpad_left) robot.intake.setRollerSlowReverse();
//            //else if (gamepad1.dpad_right) robot.intake.setRollerKeepIn();
//
//            if (gamepad1.right_trigger > 0.2) {
//                if (!didToggleIntake) {
//                    if (robot.intake.isRetracted()) robot.intake.extend();
//                    else robot.intake.retract();
//                    didToggleIntake = true;
//                }
//            } else {
//                didToggleIntake = false;
//            }
//            if(gamepad2.y){
//                robot.intake.retract();
//            }
//
//            if (gamepad2.dpad_up) {
//                if (!didToggleAlliance) {
//                    Globals.isRed = !Globals.isRed;
//                    didToggleAlliance = true;
//                }
//            } else {
//                didToggleAlliance = false;
//            }
//
//            if (gamepad2.dpad_right) robot.intake.unsetSlidesZero();
//
//            robot.update();
//
//            telemetry.addData("Globals.isRed", Globals.isRed);
//            telemetry.addData("Intake.targetPositionWhenExtended", robot.intake.getTargetPositionWhenExtended());
//            telemetry.addData("Intake.flipDownAngle", robot.intake.getFlipDownAngle());
//            telemetry.addData("armPos", armPos);
//            telemetry.addData("wristPos", wristPos);
//            telemetry.addData("clawOpen", clawOpen);
//
//            telemetry.update();
//        }
//
//    }
//}

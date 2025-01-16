package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class nTeleop extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        final double triggerThresh = 0.2;
        final double slidesAdjustSpeed = 0.5;
        boolean toggledAlliance = false;
        boolean speciToggled = false;
        boolean speciMode = true;
        boolean high = true;

        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();

        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){
            if (gamepad2.x) {
                if (!toggledAlliance) {
                    Globals.isRed = !Globals.isRed;
                    toggledAlliance = true;
                }
            } else {
                toggledAlliance = false;
            }

            if (gamepad1.x) {
                if (!speciToggled) {
                    speciMode = !speciMode;
                    speciToggled = true;
                }
            } else {
                speciToggled = false;
            }

            if (gamepad1.y) {
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesAdjustSpeed);
            } else if (gamepad1.a) {
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() - slidesAdjustSpeed);
            }

            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (robot.getState() == Robot.RobotState.IDLE) {
                    robot.setNextState(speciMode ? Robot.NextState.GRAB_SPECIMEN : Robot.NextState.INTAKE_SAMPLE);
                } else {
                    robot.setNextState(Robot.NextState.DONE);
                }
            } else if (rb_1.isClicked(gamepad1.right_bumper)) robot.setNextState(Robot.NextState.DEPOSIT);

            telemetry.addData("isRed", Globals.isRed);
            telemetry.addData("speciMode", speciMode);
            telemetry.addData("robot state", robot.getState());
            telemetry.addData("deposit height", robot.deposit.getDepositHeight());
            telemetry.update();
        }
    }
}

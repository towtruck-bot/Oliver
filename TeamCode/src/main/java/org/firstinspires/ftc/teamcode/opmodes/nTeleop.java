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

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();
        ButtonToggle lt_1 = new ButtonToggle();
        ButtonToggle rt_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle a_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();

        //Gamepad 2
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle y_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double triggerThresh = 0.2;
        final double slidesAdjustSpeed = 0.5;
        boolean speciToggled = false;
        boolean speciMode = true;
        boolean high = true;


        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){
            if (x_2.isToggled(gamepad2.x)) {
                Globals.isRed = !Globals.isRed;
            }

            if (x_1.isToggled(gamepad1.x)) {
                speciMode = !speciMode;
            }

            if (y_1.isHeld(gamepad1.y, 5)) {
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + slidesAdjustSpeed);
            } else if (a_1.isHeld(gamepad1.a, 5)) {
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

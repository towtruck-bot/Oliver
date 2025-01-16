package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class nTeleop extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        final double triggerThresh = 0.2;
        boolean toggledAlliance = false;

        boolean inDeposit = false;
        boolean speciMode = true;
        boolean high = true;

        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){
            if(gamepad2.x && !toggledAlliance){
                Globals.isRed = !Globals.isRed;
                toggledAlliance = true;
            }else{
                toggledAlliance = false;
            }

            if(gamepad1.y){
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() + 3);
            }

            if(gamepad1.a){
                robot.deposit.setDepositHeight(robot.deposit.getDepositHeight() - 3);
            }
        }
    }
}

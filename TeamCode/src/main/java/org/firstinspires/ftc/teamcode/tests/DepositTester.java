package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp(group = "Test")
public class DepositTester extends LinearOpMode {

    public void runOpMode(){
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);

        //now the goofy stuff starts
        while(opModeInInit()){
            robot.update();
        }

        /*
        a -> start transfer
        b -> start outtake
        x -> line claw up to pick up specimen off wall(1), pick up specimen off wall(2)
        y -> begin specimen deposit and get to clip position(1), let go of specimen(2)

        ignoring sample deposit bc no slides
         */

        while(!isStopRequested()){
            robot.drivetrain.drive(gamepad1);

            if(gamepad1.a){
                robot.deposit.startTransfer();
            }

            if(gamepad1.b){
                robot.deposit.startOuttake();
            }else if(robot.deposit.isOuttakeDone()){
                robot.deposit.retract();
            }

            if(gamepad1.x){
                robot.deposit.grabSpecimen();
            }else if(robot.deposit.state == Deposit.State.SPECIMEN_GRAB_WAIT && gamepad1.x){
                robot.deposit.finishSpecimenGrab();
            }

            if(gamepad1.y){
                robot.deposit.startSpecimenDeposit();
            }else if(robot.deposit.state == Deposit.State.SPECIMEN_RAISE_WAIT && gamepad1.y){
                robot.deposit.state = Deposit.State.RELEASE;
            }
        }
    }
}

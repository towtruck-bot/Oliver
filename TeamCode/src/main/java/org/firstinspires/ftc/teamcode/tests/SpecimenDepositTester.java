package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@Disabled
@Config
@TeleOp

public class SpecimenDepositTester extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;

        waitForStart();

        int currentStep = 0;

        while(!isStopRequested()){
/*
            switch(currentStep){
                case 0:
                    robot.deposit.retract();
                    if(robot.deposit.isRetractDone()){
                        currentStep++;
                    }
                    break;
                case 1:
                    robot.deposit.startTransfer();
                    if(robot.deposit.isSampleReady()){
                        currentStep++;
                    }
                    break;
                case 2:
                    robot.deposit.startOuttake();
                    if(robot.deposit.isOuttakeDone()){
                        currentStep++;
                    }
                    break;
                case 3:
                    robot.deposit.grabSpecimen();
                    if(robot.deposit.isSpecimenReady()){
                        currentStep++;
                    }
                    break;
                case 4:
                    robot.deposit.startSpecimenDeposit();
                    if(robot.deposit.isSpecimenDepositDone()){
                        currentStep++;
                    }
                    break;
                case 5:
                    robot.deposit.retract();
                    if(robot.deposit.isRetractDone()){
                        robot.deposit.state = Deposit.State.IDLE;
                    }
                    break;
            }
 */
            robot.update();
        }
    }
}

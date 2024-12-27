package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;

public class DepositTest extends LinearOpMode {
    public static Deposit.State state = Deposit.State.IDLE;
    public static boolean force = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (force) {
                robot.deposit.state = state;
            }
            robot.update();
        }

    }
}

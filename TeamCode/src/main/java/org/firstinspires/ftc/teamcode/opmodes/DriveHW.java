package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.R;

public class DriveHW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightFrontMotor;
        DcMotor leftFrontMotor;
        DcMotor rightRearMotor;
        DcMotor leftRearMotor;
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");


        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double LF = y + x + turn;
            double LR = y - x + turn;
            double RF = y - x - turn;
            double RR = y + x - turn;
            if ((abs(y) + abs(x) + abs(turn))>1){
                LF = LF/(abs(y) + abs(x) + abs(turn));
                LR = LR/(abs(y) + abs(x) + abs(turn));
                RF = RF/(abs(y) + abs(x) + abs(turn));
                RF = RF/(abs(y) + abs(x) + abs(turn));
            }

            leftFrontMotor.setPower(LF);
            leftRearMotor.setPower(LR);
            rightFrontMotor.setPower(RF);
            rightRearMotor.setPower(RR);

        }
    }
}

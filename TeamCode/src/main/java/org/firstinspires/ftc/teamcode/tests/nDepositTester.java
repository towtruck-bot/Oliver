package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class nDepositTester extends LinearOpMode {
    public static nDeposit.State stateDashboard = nDeposit.State.IDLE;
    public static nDeposit.State state = nDeposit.State.IDLE;
    public static boolean setState = false, preserveState = false;

    public static boolean coords = false;
    public static double targetY = 0, targetArm = 0, targetClaw = 0;

    public static boolean setTransferStart = false;
    public static boolean setTransferFinish = false;
    public static boolean sampleDepo = false;
    public static boolean outtake = false;
    public static boolean speciIntake = false;
    public static boolean reqGrab = false;
    public static boolean speciDepo = false;
    public static boolean release = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            if(setState){
                robot.ndeposit.state = stateDashboard;
                setState = !setState;
            }else if(preserveState){
                robot.ndeposit.state = state;
            }

            if(setTransferStart){
                robot.ndeposit.startTransfer();
                setTransferStart = !setTransferStart;
            }

            if(setTransferFinish){
                robot.ndeposit.finishTransfer();
                setTransferFinish = !setTransferFinish;
            }

            if(sampleDepo){
                robot.ndeposit.startSampleDeposit();
                sampleDepo = !sampleDepo;
            }

            if(outtake){
                robot.ndeposit.outtake();
                outtake = !outtake;
            }

            if(speciIntake){
                robot.ndeposit.startSpecimenIntake();
                speciIntake = !speciIntake;
            }

            if(reqGrab){
                robot.ndeposit.grab();
                reqGrab = !reqGrab;
            }

            if(speciDepo){
                robot.ndeposit.startSpecimenDeposit();
                speciDepo = !speciDepo;
            }

            if(release){
                robot.ndeposit.deposit();
                release = !release;
            }

            if(coords){
                robot.ndeposit.setByCoords(targetArm, targetClaw, targetY);
            }

            robot.update();
            TelemetryUtil.packet.put("current state", robot.ndeposit.state);
        }
    }
}

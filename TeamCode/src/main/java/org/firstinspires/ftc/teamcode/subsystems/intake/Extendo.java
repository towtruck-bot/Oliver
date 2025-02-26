package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Extendo {
    public static double maxExtendoLength = 27.0;

    private Robot robot;
    public PriorityMotor extendoMotor;
    private DcMotorEx m;

    private double extendoCurrentPos;
    private double targetLength = 0.0;
    public static PID extendoPID = new PID(0.15, 0.05, 0.008);
    public static double tollerance = 0.4;
    public static double slidesForcePullPow = -0.2;

    public Extendo(Robot robot){
        this.robot = robot;

        m = robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");

        if(Globals.RUNMODE != RunMode.TELEOP){
            resetExtendoEncoders();
        }

        extendoMotor = new PriorityMotor(new DcMotorEx[] {m}, "intakeExtensionMotor", 3, 5, new double[] {1}, robot.sensors);
        robot.hardwareQueue.addDevice(extendoMotor);
    }

    public void update(){
        this.extendoCurrentPos = this.robot.sensors.getExtendoPos();

        double pow = 0;

        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            extendoPID.update(0, -1.0, 1.0);
            extendoPID.resetIntegral();
            extendoMotor.setTargetPower(0.0);
        } else {
            if (this.inPosition()) {
                extendoPID.update(0, -1.0, 1.0);
                extendoPID.resetIntegral();
                pow = this.targetLength <= tollerance && this.extendoCurrentPos > 0.0 ? slidesForcePullPow : 0;
            } else {
                pow = extendoPID.update(this.targetLength - this.extendoCurrentPos, -0.7, 0.7);
            }

            this.extendoMotor.setTargetPower(pow);
        }

        TelemetryUtil.packet.put("ClawIntake extendo power", pow);
        TelemetryUtil.packet.put("ClawIntake.extendoTargetPos", this.targetLength);
        TelemetryUtil.packet.put("ClawIntake.extendoCurrentPos", this.extendoCurrentPos);
    }

    public void setTargetLength(double l){
        targetLength = Utils.minMaxClip(l, 0.0, maxExtendoLength);
    }

    public boolean inPosition() {
        if (targetLength <= tollerance) return extendoCurrentPos <= tollerance;
        return Math.abs(targetLength - extendoCurrentPos) <= tollerance;
    }

    public double getLength() {
        return extendoCurrentPos;
    }

    public void resetExtendoEncoders(){
        Log.e("RESETTTING", "RESTETING SLIDES *************");

        m.setPower(0.0);

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetLength = 0.0;
        m.setPower(0.0);
    }
}

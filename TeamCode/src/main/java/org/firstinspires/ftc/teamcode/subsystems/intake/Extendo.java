package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Extendo {
    public static double maxExtendoLength = 19.0;

    private final Robot robot;
    public PriorityMotor extendoMotor;
    private final DcMotorEx m;

    private double extendoCurrentPos = 0.0;
    private double targetLength = 0.0;
    public static PID extendoPID = new PID(0.2, 0, 0.001);
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
                pow = extendoPID.update(this.targetLength - this.extendoCurrentPos, -1.0, 1.0);
            }

            this.extendoMotor.setTargetPower(pow);
        }

        TelemetryUtil.packet.put("Extendo Power", pow);
        TelemetryUtil.packet.put("Extendo Target Length", targetLength);
        TelemetryUtil.packet.put("Extendo Current Length", extendoCurrentPos);
        TelemetryUtil.packet.put("Extendo inPosition", this.inPosition());
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

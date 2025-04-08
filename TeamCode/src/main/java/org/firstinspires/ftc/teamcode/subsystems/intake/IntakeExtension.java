package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class IntakeExtension {
    public static double maxExtendoLength = 19.5; // Actually 19

    private final Robot robot;
    public PriorityMotor extendoMotor;
    private final DcMotorEx m;

    private double extendoCurrentPos = 0.0;
    private double targetLength = 0.0;
    public static PID extendoPID = new PID(0.2, 0.05, 0.005);
    public static double slidesTolerance = 0.6;
    public static double slidesDeadZone = 0.2;
    public static double slidesKeepInPow = -0.25;
    public static double slidesForcePullPow = -0.8;
    private boolean forcePull = false;

    public IntakeExtension(Robot robot) {
        this.robot = robot;

        m = robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor");

        extendoMotor = new PriorityMotor(new DcMotorEx[] {m}, "intakeExtensionMotor", 3, 5, new double[] {1}, robot.sensors);
        robot.hardwareQueue.addDevice(extendoMotor);
    }

    public void update() {
        this.extendoCurrentPos = this.robot.sensors.getExtendoPos();

        double pow = 0;

        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            extendoPID.update(0, -1.0, 1.0);
            extendoPID.resetIntegral();
            extendoMotor.setTargetPower(0.0);
        } else {
            if (this.inPosition(slidesDeadZone)) {
                extendoPID.update(0, -1.0, 1.0);
                extendoPID.resetIntegral();
                pow = this.targetLength <= slidesTolerance && this.extendoCurrentPos >= 0.0 ? slidesKeepInPow : 0;
            } else {
                pow = extendoPID.update(this.targetLength - this.extendoCurrentPos, -0.7, 0.7);
            }

            if (forcePull) {
                pow = slidesForcePullPow;
                forcePull = false;
            }

            this.extendoMotor.setTargetPower(pow);
        }

        TelemetryUtil.packet.put("Extendo Power", pow);
        TelemetryUtil.packet.put("extendoTargetPos", targetLength);
        LogUtil.extendoTargetPos.set(targetLength);
        TelemetryUtil.packet.put("Extendo Current Length", extendoCurrentPos);
        TelemetryUtil.packet.put("Extendo inPosition", this.inPosition());
    }

    public void forcePullIn() { forcePull = true; }

    public void setTargetLength(double l) {
        if (Math.abs(l - targetLength) > 5) {
            extendoPID.resetIntegral();
        }
        targetLength = Utils.minMaxClip(l, 0.0, maxExtendoLength);
    }

    public boolean inPosition() { return inPosition(slidesTolerance); }
    public boolean inPosition(double tol) {
        if (targetLength <= tol) return extendoCurrentPos <= tol;
        return Math.abs(targetLength - extendoCurrentPos) <= tol;
    }

    public double getLength() {
        return extendoCurrentPos;
    }
}

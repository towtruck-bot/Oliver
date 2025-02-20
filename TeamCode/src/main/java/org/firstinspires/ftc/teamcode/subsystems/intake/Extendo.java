package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.renderscript.RenderScript;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Extendo {
    public static double maxVel = 1.6528571428571428;
    public static double kP = 2;
    public static double kA = 20;
    public static double kStatic = 0.05;
    public static double minPower = 0.19;
    public static double minPowerThresh = 0.5;
    public static double forceDownPower = -0.45;
    public static double forceDownThresh = 5;
    public static double maxExtendoLength = 27.0;

    private Robot robot;
    private PriorityMotor extendoMotor;
    private double length;
    private double vel;

    private double targetLength = 0.0;
    private DcMotorEx m;

    private boolean manualMode = false;

    public Extendo(Robot robot){
        this.robot = robot;

        m = robot.hardwareMap.get(DcMotorEx.class, "extendoMotor");

        if(Globals.RUNMODE != RunMode.TELEOP){
            resetExtendoEncoders();
        }

        extendoMotor = new PriorityMotor(new DcMotorEx[] {m}, "extendoMotor", 3, 5, new double[] {1}, robot.sensors);
        robot.hardwareQueue.addDevice(extendoMotor);
    }

    public void update(){
        length = robot.sensors.getExtendoPos();
        vel = robot.sensors.getExtendoVel();

        if(!manualMode){
            double pow = feedforward();
            extendoMotor.setTargetPower(Utils.minMaxClip(pow, -1, 1));
            TelemetryUtil.packet.put("Slides:: Power", pow);
        }
    }

    private double feedforward() {
        double error = targetLength - length;

        TelemetryUtil.packet.put("Extendo: Error", error);
        TelemetryUtil.packet.put("Extendo: Target", targetLength);
        TelemetryUtil.packet.put("Extendo: Length", length);

        if (targetLength <= 0.5 && length <= forceDownThresh) {
            return length <= 0.5 ? forceDownPower / 2 : forceDownPower;
        }
        return (error * (maxVel / kA)) * kP + kStatic + ((Math.abs(error) > minPowerThresh) ? minPower * Math.signum(error) : 0);
    }

    public void setTargetLength(double l){
        targetLength = Utils.minMaxClip(length, 0.0, maxExtendoLength);
    }

    public boolean inPosition(double threshold) {
        if (targetLength <= threshold) return length <= threshold;
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength() {
        return length;
    }

    public double getMaxLength() {
        return maxExtendoLength;
    }

    public void setTargetPowerFORCED(double power) {
        extendoMotor.setTargetPower(Math.max(Math.min(power, 1), -1));
    }

    public void turnOffPowerFORCED() {
        extendoMotor.motor[0].setPower(0);
    }

    public void resetExtendoEncoders(){
        Log.e("RESETTTING", "RESTETING SLIDES *************");

        m.setPower(0.0);

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetLength = 0.0;
        m.setPower(0.0);
    }

    public void setSlidesMotorsToCoast() {
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSlidesMotorsToBrake() {
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

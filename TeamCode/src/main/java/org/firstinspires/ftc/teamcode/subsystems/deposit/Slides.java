package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    public static double maxVel = 1.6528571428571428;

    //kStatic -> 0.05, kP -> 0.11
    public static double kP = 0.5; // used to be 0.11
    public static double kA = 20;
    public static double kStatic = 0.05;
    public static double minPower = 0.19;
    public static double minPowerThresh = 0.5;
    public static double forceDownPower = -0.45;
    public static double forceDownThresh = 5;
    public static double maxSlidesHeight = 33.5;

    public final PriorityMotor slidesMotors;
    private final Robot robot;

    public double length;
    public double vel;

    private double targetLength = 0;
    private DcMotorEx m1;
    private DcMotorEx m2;
    private Drivetrain drivetrain;

    public Slides(Robot robot) {
        this.robot = robot;
        this.drivetrain = robot.drivetrain;

        m1 = robot.hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        m2 = robot.hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Globals.RUNMODE != RunMode.TELEOP) {
            resetSlidesEncoders();
        }

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 3, 5, new double[] {1, 1}, robot.sensors);
        robot.hardwareQueue.addDevice(slidesMotors);
    }

    public void resetSlidesEncoders() {
        Log.e("RESETTTING", "RESTETING SLIDES *************");

        drivetrain.resetSlidesMotorRightFront();

        m1.setPower(0);
        m2.setPower(0);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetLength = 0;
        m1.setPower(0);
        m2.setPower(0);
    }

    public void setSlidesMotorsToCoast() {
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSlidesMotorsToBrake() {
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Pretty misleading function name -- Eric
     * @return power
     */
    private double feedforward() {
        double error = targetLength - length;
        TelemetryUtil.packet.put("Slides: Error", error);
        TelemetryUtil.packet.put("Slides: Target", targetLength);
        TelemetryUtil.packet.put("Slides: Length", length);

        if (targetLength <= 0.5 && length <= forceDownThresh) { // force down
            return length <= 0.5 ? forceDownPower / 2 : forceDownPower;
        }
        return (error * (maxVel / kA)) * kP + kStatic + ((Math.abs(error) > minPowerThresh) ? minPower * Math.signum(error) : 0);
    }

    public boolean manualMode = false;

    public void update() {
        length = this.robot.sensors.getSlidesPos();
        vel = this.robot.sensors.getSlidesVel();

        if (!manualMode) {
//            if (!(Globals.RUNMODE == RunMode.TESTER)) {
            double pow = feedforward();
            TelemetryUtil.packet.put("Slides: Power", pow);
            slidesMotors.setTargetPower(Math.max(Math.min(pow, 1), -1));
//            }
        }
    }

    public void setTargetLength(double length) {
        targetLength = Math.max(Math.min(length, maxSlidesHeight),0);
    }

    public void setTargetPowerFORCED(double power) {
        slidesMotors.setTargetPower(Math.max(Math.min(power, 1), -1));
    }

    public void turnOffPowerFORCED() {
        slidesMotors.motor[0].setPower(0);
        slidesMotors.motor[1].setPower(0);
    }

    public void setTargetLengthFORCED(double length) {
        targetLength = length;
    }

    public boolean inPosition(double threshold) {
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength() {
        return length;
    }
}

package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    //public static double maxVel = 55.75589743589743;
    //public static double kP = 0.8;
    //public static double kA = 46;
    public static double kStaticRamp = 0.2;
    public static double minKStaticLength = 3;
    //public static double minPower = 0.3;
    //public static double minPowerThresh = 2;
    public static double forceDownPower = -0.2;
    public static double forceDownThresh = 5;
    public static double maxSlidesHeight = 35.3;
    public static PID pid = new PID(0.3, 0.005, 0.001);
    private int concurrence = 0;
    private double slidesTargetDelta = 0;
    private boolean justZeroed = false;

    public final PriorityMotor slidesMotors;
    private final Robot robot;

    private double length;
    private double vel;

    private double targetLength = 0;
    private final DcMotorEx m1;
    private final DcMotorEx m2;

    public Slides(Robot robot) {
        this.robot = robot;

        m1 = robot.hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        m2 = robot.hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        // m2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 3, 5, new double[] {-1, 1}, robot.sensors);
        robot.hardwareQueue.addDevice(slidesMotors);
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
    /*private double feedforward() {
        double error = targetLength - length;
        TelemetryUtil.packet.put("Slides : error", error);
        TelemetryUtil.packet.put("Slides : targetLength", targetLength);
        LogUtil.slidesTargetPos.set(targetLength);
        //TelemetryUtil.packet.put("Slides: Length", length);

        if (targetLength <= 3 && length <= forceDownThresh && Math.abs(error) > 1) { // force down
            return length <= 0.5 ? forceDownPower / 2 : forceDownPower;
        }
        return (error * (maxVel / kA)) * kP + kStatic + kStaticRamp * targetLength + ((Math.abs(error) > minPowerThresh) ? minPower * Math.signum(error) : 0);
    }*/

    public boolean manualMode = false;

    public void update() {
        length = this.robot.sensors.getSlidesPos();
        vel = this.robot.sensors.getSlidesVel();
        TelemetryUtil.packet.put("Slides: Velocity", vel);

        if (!manualMode) {
//            if (!(Globals.RUNMODE == RunMode.TESTER)) {
            TelemetryUtil.packet.put("slidesError", targetLength - length);
            double pow = pid.update(targetLength - length, -1.0, 1.0) + ((length > minKStaticLength) ? kStaticRamp / maxSlidesHeight * length : 0);//feedforward();
            if (length <= forceDownThresh && targetLength == 0)
                pow = forceDownPower;

            // We can auto reset the slides position
            if (targetLength == 0 && length <= 2) ++concurrence;
            else concurrence = 0;

            if (concurrence >= 10 && slidesTargetDelta >= 10) {
                robot.sensors.softwareResetSlidesEncoders();
                justZeroed = true;
            }

            if (this.inPosition(0.3)) pid.resetIntegral();

            TelemetryUtil.packet.put("Slides : Power", pow);
            TelemetryUtil.packet.put("Slides : Target", targetLength);
            TelemetryUtil.packet.put("Slides : concurrence", concurrence);
            TelemetryUtil.packet.put("Slides : targetDelta", slidesTargetDelta);
            slidesMotors.setTargetPower(Utils.minMaxClip(pow, -1, 1));
//            }
        }
    }

    public void setTargetLength(double length) {
        double tl = Utils.minMaxClip(length, 0, maxSlidesHeight);
        if (justZeroed && tl > 0) {
            slidesTargetDelta = 0;
            justZeroed = false;
        } else
            slidesTargetDelta += Math.abs(tl - targetLength);

        targetLength = tl;
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
        if (targetLength <= threshold) {
            return length <= threshold;
        }
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength() {
        return length;
    }

    public double getTargetLength(){
        return targetLength;
    }

    public double getVel() {
        return vel;
    }
}

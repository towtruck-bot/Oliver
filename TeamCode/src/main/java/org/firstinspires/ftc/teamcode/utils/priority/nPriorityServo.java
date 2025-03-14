package org.firstinspires.ftc.teamcode.utils.priority;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class nPriorityServo extends PriorityDevice {
    public enum ServoType {
        // Radians/s is speed
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(1 / Math.toRadians(305), Math.toRadians(200) / 0.443),
        AXON_MAX(0.1775562245447108, 6.5830247235911042),
        AXON_MICRO(0.1775562245447108, 6.5830247235911042),  // TODO need to tune
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12),
        HITEC(0.2966648, 5.6403953024772129);

        public final double positionPerRadian;
        public final double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public final Servo[] servos;
    private final ServoType type;
    public final double minPos;
    public final double maxPos;
    public final double basePos;
    private double currentAngle = 0, targetAngle = 0, power = 1.0, currentIntermediateTargetAngle = 0;
    protected final boolean[] reversed;
    private long lastLoopTime = Globals.LOOP_START;
    private boolean first = true; // Priority servo has a problem when the servos won't get set at the start if theyre set to 0

    /**
     * Basic initializer
     *
     * @param servos If servos are connected in parallel add them all here
     * @param name Name of device for HardwareQueue lookup
     * @param type Type of servo type
     * @param minPos Minimum pose that it can possibly move to
     * @param maxPos Maximum pose that it can possibly move to
     * @param basePos Pose that is set t0 "0"
     * @param reversed Which servos in the servo array are reversed or not
     * @param basePriority BP
     * @param priorityScale PS
     */
    public nPriorityServo(Servo[] servos, String name, ServoType type, double minPos, double maxPos, double basePos, boolean[] reversed, double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.servos = servos;
        this.type = type;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.basePos = basePos;
        this.reversed = reversed;
        this.currentAngle = convertPosToAngle(basePos);
    }

    private double convertPosToAngle(double pos) {
        pos -= basePos;
        pos /= type.positionPerRadian;
        return pos;
    }

    private double convertAngleToPos(double ang) {
        ang *= type.positionPerRadian;
        ang += basePos;
        return ang;
    }

    public boolean inPosition(){
        return Math.abs(targetAngle-currentAngle) < Math.toRadians(0.01);
    }

    public boolean inPosition(double thresh){
        return Math.abs(targetAngle - currentAngle) < thresh;
    }

    public void setTargetAngle(double angle) {
        this.targetAngle = Utils.minMaxClip(angle, convertPosToAngle(minPos), convertPosToAngle(maxPos));
    }

    public void setTargetAngle(double angle, double power) {
        this.targetAngle = Utils.minMaxClip(angle, convertPosToAngle(minPos), convertPosToAngle(maxPos));
        this.power = power;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetPos(double pos) {
        this.setTargetPos(Utils.minMaxClip(pos, minPos, maxPos), 1);
    }

    public void setTargetPos(double pos, double power) {
        this.targetAngle = convertPosToAngle(Math.max(Math.min(pos, maxPos), minPos));
        this.power = power;
    }

    public double getTargetPos() {
        return convertAngleToPos(targetAngle);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    @Override
    protected void update() {
        Log.i("SLCI", "Updated " + name);

        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = ((double) currentTime - lastUpdateTime)/1.0E9;

        double error = targetAngle - currentAngle;
        double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);

        currentIntermediateTargetAngle += deltaAngle;

        // Clamp
        if (Math.abs(deltaAngle) > Math.abs(error) || power == 1)
            currentIntermediateTargetAngle = targetAngle;
        //Log.e("ooga", "booga");

        // Update servos
        for (int i = 0; i < servos.length; i++) {
            double pos = 0;
            if (!reversed[i]) {
                pos = convertAngleToPos(currentIntermediateTargetAngle);
            } else {
                pos = 1 - convertAngleToPos(currentIntermediateTargetAngle);
            }
            servos[i].setPosition(pos);
            Log.i("SLCI", "Set position of " + name + " to position " + pos);
        }

        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        // STUPID STUPID HACK I HATE YOU
        if (first) {
            if (!(Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER)) {
                update();
            }
            Log.i(name, currentAngle + " is the value [ Eric's Log ]");
            first = false;
        }


        if (isUpdated)
            return 0;

        // Update the servo internal values
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime)/1.0E9;

        // We actually use this to pretty much just get direction
        double error = currentIntermediateTargetAngle - currentAngle;

        // How much the servo has moved from the start of the loop to now
        double deltaAngle = loopTime * type.speed * Math.signum(error) * power;

//        Log.e("adding " + this.name + "deltaAngle" , deltaAngle + "");
//        Log.e(this.name + "'s current angle" , currentAngle + "");
//        Log.e(this.name + "_loopTime" , loopTime + "");
//        Log.e(this.name + "_type.speed" , type.speed + "");
//        Log.e(this.name + "_error" , error + "");
//        Log.e(this.name + "_power" , power + "");
//        Log.e(this.name + "_targetAngle" , targetAngle + "");
//        Log.e(this.name + "_currentAngle" , currentAngle + "");
//        Log.e(this.name + "_currentIntermediateTargetAngle" , currentIntermediateTargetAngle + "");

        currentAngle += deltaAngle;

        // Clamp
        if (Math.abs(deltaAngle) > Math.abs(error))
            currentAngle = targetAngle;

        lastLoopTime = currentTime;

        // Dawg what the hell??
        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        // Ong trust this function.
        if (currentAngle - targetAngle == 0) {
            Log.e("beh", "god damnit");
        }
        double priority = (((currentAngle - targetAngle) != 0) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;

        // Yuh that means it just updated. Dont even touch that thing
        if (priority == 0) {
            lastUpdateTime = System.nanoTime();
        }

        return priority;
    }
}

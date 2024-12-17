package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Globals;

public class nPriorityServo extends PriorityDevice {
    public enum ServoType {
        // Radians/s is speed
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(0.1784612002049795, 5.6403953024772129),
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

    private final Servo[] servos;
    private final ServoType type;
    private final double minPos, maxPos;
    private final double basePos;
    private double currentAngle = 0, targetAngle = 0, power = 0, currentIntermediateTargetAngle = 0;
    protected final boolean[] reversed;
    private long lastLoopTime = Globals.LOOP_START;

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

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetPos(double pos) {
        this.targetAngle = convertPosToAngle(Math.max(Math.min(pos, 1), 0));
    }

    public double getTargetPos() {
        return convertAngleToPos(targetAngle);
    }

    @Override
    protected void update() {
        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = ((double) currentTime - lastUpdateTime)/1.0E9;

        double deltaAngle = timeSinceLastUpdate * type.speed;

        currentIntermediateTargetAngle += deltaAngle;

        // Clamp
        if (currentIntermediateTargetAngle > targetAngle)
            currentIntermediateTargetAngle = targetAngle;

        if (power == 1) // Fastest we can go; send it
            currentIntermediateTargetAngle = targetAngle;

        // Update servos
        for (int i = 0; i < servos.length; i++) {
            double pos = 0;
            if (!reversed[i]) {
                pos = convertAngleToPos(currentIntermediateTargetAngle);
            } else {
                pos = (1 - basePos) - convertAngleToPos(currentIntermediateTargetAngle);
            }
            servos[i].setPosition(pos);
        }

        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (isUpdated)
            return 0;

        // Update the servo internal values
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime)/1.0E9;

        // We actually use this to pretty much just get direction
        double error = currentIntermediateTargetAngle - currentAngle;

        // How much the servo has moved from the start of the loop to now
        double deltaAngle = loopTime * type.speed * Math.signum(error);

        currentAngle += deltaAngle;

        // Clamp
        if (currentAngle > targetAngle)
            currentAngle = targetAngle;

        lastLoopTime = currentTime;

        // Dawg what the hell??
        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        // Ong trust this function.
        double priority = (((currentAngle - targetAngle) == 0) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;

        // Yuh that means it just updated. Dont even touch that thing
        if (priority == 0) {
            lastUpdateTime = System.nanoTime();
        }

        return priority;
    }
}

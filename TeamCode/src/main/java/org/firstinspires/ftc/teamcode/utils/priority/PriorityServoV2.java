package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.Servo;

public class PriorityServoV2 extends PriorityDevice {
    public enum ServoType {
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

        public double positionPerRadian;
        public double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public Servo[] servo;
    public String name;
    public ServoType type;
    protected double[] directions;
    public double minPos, maxPos, minAng, maxAng, basePos;
    protected double currentAngle = 0.0, targetAngle = 0.0, power = 0.0;
    protected  double currentIntermediateTargetAngle = 0.0;
    public double slowDownDist, slowDownPow;
    protected boolean reachedIntermediate = false;
    private long lastLoopTime = System.nanoTime();

    public PriorityServoV2(Servo[] servo, String name, ServoType type, double loadMultiplier, double[] directions, double min, double max, double basePos, double slowDownDist, double slowDownPow, double basePriority, double priorityScale){
        super(basePriority, priorityScale, name);
        this.servo = servo;
        this.name = name;
        this.type = type;
        this.type.speed *= loadMultiplier;
        this.directions = directions;

        this.basePos = basePos;

        minPos = Math.min(min, max);
        maxPos = Math.max(min, max);

        minAng = convertPosToAngle(minPos);
        maxAng = convertPosToAngle(maxPos);

        this.slowDownDist = slowDownDist;
        this.slowDownPow = slowDownPow;
    }

    public double convertPosToAngle(double pos){
        return (pos - basePos) / type.positionPerRadian;
    }

    public double convertAngleToPos(double ang){
        return ang * type.positionPerRadian - basePos;
    }

    public void updateServoValues(){
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime);

        //error = dist to intermediate point(based on last update time), finalError = dist to final end point
        double error = currentIntermediateTargetAngle - currentAngle;
        double finalError = targetAngle - currentAngle;

        //calculate change in angle from last update, use finalError to determine if power needs to be changed
        //Q: math wise, how does power fit into this equation? can we actually just multiply? -- James
        double deltaAngle = loopTime * type.speed * (Math.abs(finalError) <= slowDownDist ? slowDownPow : power) * Math.signum(error);

        //if moved sufficient distance between last update and present, set detalAngle to error as not to overshoot
        if(Math.abs(deltaAngle) > Math.abs(error)){
            deltaAngle = error;
        }
        currentAngle += deltaAngle;
        lastLoopTime = currentTime;
    }

    @Override
    protected void update() {
        long currentTime = System.nanoTime();
        double updateTime = ((double) currentTime - lastUpdateTime) / 1.0E9;

        //error = distance to end point
        double error = targetAngle - currentAngle;
        //calculate change in angle from last update, use error to determine if power needs to be changed
        double deltaAngle = updateTime * type.speed * (Math.abs(error) <= slowDownDist ? slowDownPow : power) * Math.signum(error);

        //if moved sufficient distance between last update and present, set detalAngle to error as not to overshoot
        if(Math.abs(deltaAngle) > Math.abs(error)){
            deltaAngle = error;
        }

        currentIntermediateTargetAngle += deltaAngle;

        double distToSlowDown = Math.abs(error) - convertPosToAngle(slowDownDist);
        //check if a cap needs to be added so slowDownDist can kick in, specifically if there is less than 15 deg left
        if(slowDownDist != 0 && (Math.signum(distToSlowDown) == 1.0 && distToSlowDown < Math.toRadians(15))){
            currentIntermediateTargetAngle = targetAngle - slowDownDist * Math.signum(error); // account for slowdown, stop right before boundary
        }else if(power == 1.0 || slowDownDist == 0){
            currentIntermediateTargetAngle = targetAngle; // if no slowdown -> all the way to end
        }

        //if forward, move to intermediate angle. if reversed, move in the negative direction from basePos
        for(int i = 0; i < servo.length; i++){
            if(directions[i] == 1){
                servo[i].setPosition(convertAngleToPos(currentIntermediateTargetAngle));
            }else{
                servo[i].setPosition(convertAngleToPos(basePos - convertAngleToPos(currentIntermediateTargetAngle)));
            }
        }

        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if(isUpdated){
            return 0;
        }

        updateServoValues();

        if(timeRemaining * 1000.0 <= callLengthMillis / 2.0){
            return 0;
        }

        //@Kyle gonna need an explanation for this one --James
        double priority = ((reachedIntermediate && currentIntermediateTargetAngle != targetAngle) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;

        if(priority == 0.0){
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        return priority;
    }

    public void setCurrentAngle(double currentAngle){
        this.currentAngle = currentAngle;
    }

    public double getCurrentAngle(){return this.currentAngle;}

    public boolean inPosition(){
        return Math.abs(targetAngle - currentAngle) < Math.toRadians(0.01);
    }

    //Q: Why do we not return targetAngle but return the intermediate? -- James
    public double getTargetPosition(){
        return convertAngleToPos(currentIntermediateTargetAngle);
    }

    public double getTargetAngle(){
        return currentIntermediateTargetAngle;
    }

    public void setTargetAngle(double target){
        this.targetAngle = target;
    }
}

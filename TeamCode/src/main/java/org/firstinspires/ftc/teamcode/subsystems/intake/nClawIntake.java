package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Config
public class nClawIntake {
    private final Robot robot;

    public final IntakeTurret intakeTurret;

    private final REVColorSensorV3 colorSensorV3;

    public final DigitalChannel intakeLight;

    private double intakeSetTargetPos;

    // turretBufferAng -> angle that allows for any rotation to occur with the turret still inside the robot. use in any retract/extend states

    public static double transferClawRotation = 0;
    public static double hoverAngle = 2.0515;
    public static double turretRetractedAngle = -0.3769, turretSearchAngle = 1.65, turretTransferAngle = 0.1146, turretGrabAngle = 2.5133;
    public static double turretTransferRotation = 3.165;
    public static double minExtension = 2; // What we require before giving full range of motion
    private long hoverStart = 0;
    public static double hoverDelay = 150;
    public static double transferExtension = 0;
    public static double turretSearchRotation = 3.165;

    private boolean grab = false;
    private boolean sampleStatus = false;
    //private boolean useCamera = true;
    private boolean finishTransferRequest = false;
    private boolean extendRequest = false;
    public Pose2d target;
    //public boolean manualGrab = false;
    public static int blockPickUpPSThreashold = 263;
    private int psReads = 0;
    private int consecutivePSPositives = 0;
    private Target targetType = Target.RELATIVE;
    private GrabMethod grabMethod = GrabMethod.CONTINUOUS_SEARCH_MG;
    private Pose2d known = null;
    private boolean retryGrab = false;

    private double manualTurretAngle, manualClawAngle;

    public enum Target {
        RELATIVE,
        GLOBAL,
        MANUAL
    }

    public enum GrabMethod {
        MANUAL_AIM(false, true),
        MANUAL_TARGET(false, true),
        CONTINUOUS_SEARCH_MG(true, true),
        SEARCH_HOVER_MG(true, true),
        AUTOGRAB(true, false);

        private final boolean useCamera, manualGrab;

        GrabMethod(boolean useCamera, boolean manualGrab) {
            this.useCamera = useCamera;
            this.manualGrab = manualGrab;
        }
    }

    public enum State {
        SEARCH,
        HOVER,
        LOWER,
        GRAB_CLOSE,
        START_RETRACT,
        RETRACT,
        READY,
        TRANSFER_WAIT,
        TRANSFER_END,
        TEST
    }

    public State state = State.READY;

    public nClawIntake(Robot robot) {
        this.robot = robot;

        intakeTurret = new IntakeTurret(this.robot);

        intakeLight = robot.hardwareMap.get(DigitalChannel.class, "intakeLight");
        colorSensorV3 = robot.hardwareMap.get(REVColorSensorV3.class, "intakeClawColorSensor");
        colorSensorV3.configurePS(REVColorSensorV3.PSResolution.EIGHT, REVColorSensorV3.PSMeasureRate.m6p25s);
        colorSensorV3.sendControlRequest(new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.PROX_SENSOR_ENABLED)
        );
        intakeLight.setMode(DigitalChannel.Mode.OUTPUT);
        intakeLight.setState(false);

        intakeSetTargetPos = 15;

        target = new Pose2d(0, 0, 0);
//        intakeTurret.turretArm.servos[0].getController().pwmEnable();
    }

    //general update for entire class
    public void update() {
        switch (state) {
            /*case START_EXTEND:
                // Pre-rotate the turret + claw servos
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setTurretArmTarget(turretBufferAngle);
                intakeTurret.setTurretRotation(turretPreRotation);
                intakeTurret.setClawRotation(0);

                intakeTurret.setClawState(grab);

                // Wait for extension past certain length or for the buffer ang to be reached, meaning we can full send rotation
                if (intakeTurret.turretAngInPosition()) {
                    //if (intakeSetTargetPos > minExtension)
                        state = State.FULL_EXTEND;
                    //else
                    //    state = State.MID_EXTEND;
                }
                break;
            case MID_EXTEND:
                // Get past the side plates so if our target position is less than min extension we won't clip
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setTurretArmTarget(turretBufferAngle);
                intakeTurret.setTurretRotation(turretPastSidePlatesRotation);
                intakeTurret.setClawRotation(0);

                if (intakeTurret.turretRotInPosition())
                    state = State.FULL_EXTEND;
                break;
            case FULL_EXTEND:
                // Fully extend + rotate to search positions
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                intakeTurret.setTurretArmTarget(turretSearchAngle);
                intakeTurret.setTurretRotation(turretSearchRotation);
                intakeTurret.setClawRotation(0);

                intakeTurret.setClawState(grab);

                // Wait for full extension and turret in position before starting search
                if (intakeTurret.extendoInPosition(1.0) && intakeTurret.turretRotInPosition()) {
                    grab = false;
                    sampleStatus = false;
                    if (grabMethod.useCamera) {
                        state = State.SEARCH;
                        robot.vision.startDetection();
                        robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
                        intakeLight.setState(true);
                    } else {
                        hoverStart = System.currentTimeMillis();
                        state = State.HOVER;
                    }
                }
                break;*/
            case SEARCH:
                aimAtKnown();

                robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
                robot.vision.setNewOrientation(intakeTurret.getTurretRotation() - Math.PI);

                intakeTurret.setClawState(false);

                Log.i("CHECKING IT", robot.vision.isStable() + " stable");
                Log.i("CHECKING IT", robot.vision.gottenFirstContact() + " first contact");
                Log.i("CHECKING IT", intakeTurret.rotInPosition() + " rot in pos");
                switch (grabMethod) {
                    case CONTINUOUS_SEARCH_MG:
                        if (!grab)
                            break;
                    case SEARCH_HOVER_MG:
                    case AUTOGRAB:
                        if (!(robot.vision.isStable() && robot.vision.gottenFirstContact() && intakeTurret.inPosition() && intakeTurret.extendoInPosition()))
                            break;

                        hoverStart = System.currentTimeMillis();
                        Pose2d p = robot.vision.getBlockPos();
                        target = new Pose2d(
                            p.x,
                            p.y,
                            -p.heading
                        );
                        targetType = Target.RELATIVE; // Kind of needed here or else its weird
                        robot.vision.stopDetection();
                        state = State.HOVER;
                        break;
                }
                break;

            case HOVER:
                aimAtTarget();
                intakeTurret.setTurretArmTarget(hoverAngle);
                intakeTurret.setClawState(false);

                if (intakeTurret.inPosition(Math.toRadians(10)) && (grabMethod == GrabMethod.MANUAL_AIM || System.currentTimeMillis() - hoverStart > hoverDelay)) {
                    if (!grabMethod.manualGrab || grab) {
                        state = State.LOWER;
                    }
                }

                break;

            case LOWER: // Slam it down sometimes so we need to hover
                aimAtTarget();

                intakeTurret.setClawState(false);

                // everything in position before grabbing
                if (intakeTurret.inPosition(Math.toRadians(2))) {
                    consecutivePSPositives = psReads = 0;
                    state = State.GRAB_CLOSE;
                }
                break;
            case GRAB_CLOSE:
                aimAtTarget();

                if (grabMethod == GrabMethod.MANUAL_AIM) {
                    sampleStatus = grab;
                    intakeTurret.setClawState(grab);
                    if (intakeTurret.clawInPosition()) intakeTurret.setTurretArmTarget(hoverAngle);
                    if (!grab) state = State.HOVER;
                    break;
                }

                if (psReads >= 35) {
                    if (!retryGrab) grab = false;
                    if (grabMethod.useCamera) {
                        state = State.SEARCH;
                        robot.vision.startDetection();
                        intakeLight.setState(true);
                    } else {
                        hoverStart = System.currentTimeMillis();
                        state = State.HOVER;
                    }
                    break;
                }

                int val = colorSensorV3.readPS();
                Log.e("colorSensorPS Value", val + "");
                if (val > blockPickUpPSThreashold) {
                    consecutivePSPositives++;
                } else
                    consecutivePSPositives = 0;
                psReads++;

                // grab
                grab = true;
                intakeTurret.setClawState(grab);

                // begin retract once grab finished
                if (consecutivePSPositives >= 20) {
                    sampleStatus = true;
                }

                if (sampleStatus && intakeTurret.clawInPosition()) {
                    known = null;
                    state = State.START_RETRACT;
                    robot.ndeposit.startTransfer();
                    intakeLight.setState(false);
                }

                break;
            case START_RETRACT:
                // Get the arm in a proper angle for a full retract
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(grab);

                // if grab failed go back to search
                if (intakeTurret.turretAngInPosition(Math.toRadians(30)) && intakeTurret.turretRotInPosition(Math.toRadians(30))) {
                    // If we have a sample, transfer otherwise just retract into it
                    if (sampleStatus)
                        state = State.TRANSFER_WAIT;
                    else {
                        state = State.RETRACT;
                        grab = false;
                        if (robot.ndeposit.state == nDeposit.State.TRANSFER_BUFFER) robot.ndeposit.returnToIdle();
                    }
                }
                break;
            case RETRACT:
                // full retract into transfer
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // true grab -> holding a sample
                if (intakeTurret.extendoInPosition()) {
                    state = State.READY;
                    this.intakeLight.setState(false);
                }

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                break;
            case READY:
                // hold in start position, everything tucked in while moving so defense can be played. no sample ver
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                break;
            case TRANSFER_WAIT:
                // hold in transfer position
                intakeTurret.setIntakeExtension(transferExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(true);

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                // Complete transfer can only be called in TRANSFER_WAIT, must have everything correct
                // used to release intake grip on sample, should be called in deposit after the deposit has a firm grip
                // TODO: check endAffector.inPosition()
                else if (finishTransferRequest && intakeTurret.inPosition() && intakeTurret.extendoInPosition() && robot.ndeposit.isHolding()) {
                    state = State.TRANSFER_END;
                    finishTransferRequest = false;
                    sampleStatus = false;
                }
                break;
            case TRANSFER_END:
                intakeTurret.setIntakeExtension(transferExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // once the grab is finished, send back to RETRACT. false grab changes from HOLD to READY
                // no need to worry about whacking stuff b/c both states require rotation to be in the turretTransferRot value
                if (intakeTurret.grabInPosition() && robot.ndeposit.retractReady()) {
                    state = State.RETRACT;
                    grab = false;
                }
                break;

            case TEST:
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        intakeTurret.update();
        updateTelemetry();
    }

    public void finishTransfer() {
        finishTransferRequest = state == State.TRANSFER_WAIT;
    }

    public void setKnownIntakePose(Pose2d k) {
        known = k.clone();
    }

    public void removeKnown() {
        known = null;
    }

    public void setTargetPose(Pose2d t) {
        target = t.clone();
    }

    public void extend() {
        extendRequest = true;
    }

    public boolean isOut() {
        return state == State.SEARCH || state == State.HOVER || state == State.LOWER ||  state == State.GRAB_CLOSE;
    }

    public boolean isExtended() {
        return isOut() && intakeTurret.extendoInPosition();
    }

    public void retract() {
        if (state != State.READY) {
            this.state = State.START_RETRACT;
            intakeLight.setState(false);
            if (grab && grabMethod == GrabMethod.MANUAL_AIM) robot.ndeposit.startTransfer();
        }
    }

    public boolean isRetracted() {
        return state == State.READY;
    }

    public void setGrab(boolean closed) {
        grab = closed;
    }

    // Confirm pickup
    public void confirmGrab() {
        sampleStatus = true;
    }

    public boolean hasSample() {
        return sampleStatus;
    }

    public boolean isTransferReady() {
        RobotLog.e("TSPMO " + intakeTurret.inPosition() + " " + intakeTurret.extendoInPosition() + " " + (state == State.TRANSFER_WAIT));
        return intakeTurret.inPosition() && intakeTurret.extendoInPosition() && state == State.TRANSFER_WAIT;
    }

    /*public void release() {
        if (this.state == State.HOLD) {
            this.state = State.READY;
            grab = false;
        }
    }*/

    //public boolean dropFinished() {
    //    return state == State.SEARCH;
    //}

    public double getExtendoTargetPos() { return this.intakeSetTargetPos; }
    public void setExtendoTargetPos(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 1, 19);
    }
    public double getManualTurretAngle() { return this.manualTurretAngle; }
    public void setManualTurretAngle(double targetPos) {
        while (targetPos < -1.8) targetPos += 1.8 * 2;
        while (targetPos > 1.8) targetPos -= 1.8 * 2;
        this.manualTurretAngle = targetPos;
    }
    public double getManualClawAngle() { return this.manualClawAngle; }
    public void setManualClawAngle(double targetPos) {
        while (targetPos < -1.8) targetPos += 1.8 * 2;
        while (targetPos > 1.8) targetPos -= 1.8 * 2;
        this.manualClawAngle = targetPos;
    }

    public double getIntakeLength() {
        return intakeTurret.intakeExtension.getLength();
    }

    public Vector2 getIntakeRelativeToRobot() {
        return new Vector2(
            getIntakeLength() + IntakeTurret.extendoOffset + Math.sin(intakeTurret.getTurretRotation() - Math.toRadians(90)) * IntakeTurret.turretLengthLL,
            -IntakeTurret.turretLengthLL * Math.cos(intakeTurret.getTurretRotation() - Math.toRadians(90))
        );
    }

//    public void forcePullIn() { forcePull = true; }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("ClawIntake : clawRotation angle", intakeTurret.getClawRotation());
        TelemetryUtil.packet.put("ClawIntake : manualTurretAngle", manualTurretAngle);
        TelemetryUtil.packet.put("ClawIntake : manualClawAngle", manualClawAngle);
        TelemetryUtil.packet.put("ClawIntake : grab", grab);
        TelemetryUtil.packet.put("ClawIntake : grabMethod", grabMethod);
        TelemetryUtil.packet.put("ClawIntake : targetType", targetType);
        TelemetryUtil.packet.put("ClawIntake : state", this.state);
        TelemetryUtil.packet.put("intakeState", this.state);
        LogUtil.intakeState.set(this.state.toString());
        TelemetryUtil.packet.put("LL : Target X", target.x);
        TelemetryUtil.packet.put("LL : Target Y", target.y);
        TelemetryUtil.packet.put("LL : Target Heading", target.heading);

    }

    public void setGrabMethod(GrabMethod grabMethod) {
        this.grabMethod = grabMethod;
    }

    public int readPS() {
        return colorSensorV3.readPS();
    }

    public void setTargetType(Target targetType) {
        this.targetType = targetType;
    }

    public void aimAtKnown() {
        if (known != null) {
            // Begin Search, dynamic correction
            Pose2d curr = robot.sensors.getOdometryPosition();
            double deltaX = (known.x - curr.x);
            double deltaY = (known.y - curr.y);

            double relX = Math.cos(curr.heading)*deltaX + Math.sin(curr.heading)*deltaY;
            double relY = -Math.sin(curr.heading)*deltaX + Math.cos(curr.heading)*deltaY;

            intakeTurret.extendTo(new Vector2(relX, relY));
            intakeTurret.setTurretArmTarget(turretSearchAngle);
            intakeTurret.setClawRotation(target.heading);
        } else {
            // Begin Search, just hold positions
            intakeTurret.setIntakeExtension(intakeSetTargetPos);
            intakeTurret.setClawRotation(target.heading);
            intakeTurret.setTurretArmTarget(turretSearchAngle);
            intakeTurret.setTurretRotation(turretSearchRotation);
        }
    }

    public void aimAtTarget() {
        switch (targetType) {
            case RELATIVE: {
                intakeTurret.intakeAt(target);
                break;
            }
            case GLOBAL: {
                double deltaX = (target.x - robot.sensors.getOdometryPosition().x);
                double deltaY = (target.y - robot.sensors.getOdometryPosition().y);

                // convert error into direction robot is facing
                intakeTurret.intakeAt(new Pose2d(
                    Math.cos(robot.sensors.getOdometryPosition().heading) * deltaX + Math.sin(robot.sensors.getOdometryPosition().heading) * deltaY,
                    -Math.sin(robot.sensors.getOdometryPosition().heading) * deltaX + Math.cos(robot.sensors.getOdometryPosition().heading) * deltaY,
                    target.heading - robot.sensors.getOdometryPosition().heading
                ));
                break;
            }
            case MANUAL: {
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                intakeTurret.setTurretRotation(Math.PI + manualTurretAngle);
                intakeTurret.setClawRotation(manualClawAngle);
                intakeTurret.setTurretArmTarget(turretGrabAngle);
                break;
            }
        }
    }

    public void setRetryGrab(boolean retryGrab) {
        this.retryGrab = retryGrab;
    }

    private void doExtend() {
        if (grabMethod.useCamera) {
            state = State.SEARCH;
            robot.vision.startDetection();
            robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
            intakeLight.setState(true);
        } else {
            hoverStart = System.currentTimeMillis();
            state = State.HOVER;
        }
    }
}

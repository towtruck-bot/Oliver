package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    public enum IntakeRollerState {
        ON,
        OFF,
        KEEP_IN,
        UNJAM,
        REVERSE,
        SLOW_REVERSE,
    }

    public enum IntakeState {
        START_EXTENDING,
        EXTENDING,
        DROP_DOWN,
        EXTENDED,
        PICK_UP,
        RETRACTING,
        FINISH_RETRACTING,
        IDLE,
        TRANSFER,
    }

    public final Robot robot;
//    public final PriorityMotor intakeRollerMotor;
    public final PriorityMotor intakeExtensionMotor;
    public final nPriorityServo intakeFlipServo;

    public IntakeState intakeState = IntakeState.FINISH_RETRACTING;

    private IntakeRollerState intakeRollerState = IntakeRollerState.OFF;
    public static double keepBlockInPower = 0.35;
    public static double slowReversePower = 0.5;
    public static long unjamDuration = 300;
    private long unjamLastTime;
    private boolean didStopRoller = true;

    public static boolean autoRetractIntake = false;
    public static boolean autoUnjamIntake = false;

    private long lastRetractTime = -1;
    public static long retractMaxDuration = 1000;
    public static double slidesBasePos = 0;
    public static double slidesMaxPos = 22;
    public static double slidesTolerance = 1.0;
    public static double startFlipThresh = 6;
    public static double extendedMinPos = 3;
    private double targetPositionWhenExtended = 15.0;
    private double slidesCurrentPos = 0;
    private double slidesControlTargetPos = 0;
    public static PID pid = new PID(0.185, 0.002, 0.008);

    public static double flipGearRatio = -24.0 / 40.0;
    private double flipDownAngle = 165;
    public static double flipDownAngleMin = 125;
    public static double flipDownAngleMax = 185;
    public static double flipAngleToGoOverBarrier = 100;

    private Sensors.BlockColor sampleColor = Sensors.BlockColor.NONE;
    private long sampleCheckTime;
    public static long sampleConfirmDuration = 50;

    /**
     * Initializes the intake. Uses motors intakeRollerMotor and intakeExtensionMotor, servo intakeFlipServo. -- Daniel
     * @param robot the robot (must have robot.sensors defined)
     */
    public Intake(@NonNull Robot robot) {
        this.robot = robot;

//        intakeRollerMotor = new PriorityMotor(
//                this.robot.hardwareMap.get(DcMotorEx.class, "intakeRollerMotor"),
//                "intakeRollerMotor",
//                1, 3, -1.0, this.robot.sensors
//        );
//        robot.hardwareQueue.addDevice(intakeRollerMotor);

        intakeExtensionMotor = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                1, 5, this.robot.sensors
        );
        robot.hardwareQueue.addDevice(intakeExtensionMotor);

        intakeFlipServo = new nPriorityServo(
                new Servo[] {this.robot.hardwareMap.get(Servo.class, "intakeFlipServo")},
                "intakeFlipServo",
                nPriorityServo.ServoType.HITEC,
                0.15, 0.7, 0.7,
                new boolean[] {false},
                1.0, 5.0
        );
        robot.hardwareQueue.addDevice(intakeFlipServo);

        intakeFlipServo.setTargetAngle(Math.toRadians(5 * flipGearRatio), 1.0);
    }

    /**
     * Updates the motors for both roller and extension. Uses PID with deadzone for extension. -- Daniel
     */
    public void update() {
        long currentTime = System.nanoTime();
        slidesCurrentPos = this.robot.sensors.getExtendoPosition() - slidesBasePos;
        sampleColor = this.robot.sensors.getIntakeColor();

        if (Globals.TESTING_DISABLE_CONTROL && Globals.RUNMODE == RunMode.TESTER) {
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
//            intakeRollerMotor.setTargetPower(0.0);
            updateTelemetry();
            return;
        }

        // FSM
        switch (this.intakeState) {
            case START_EXTENDING:
                setRollerOff();
                slidesControlTargetPos = Math.max(targetPositionWhenExtended, Math.max(extendedMinPos, startFlipThresh));
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (this.slidesCurrentPos >= startFlipThresh - slidesTolerance) this.intakeState = IntakeState.EXTENDING;
                else break;
            case EXTENDING:
                setRollerOff();
                slidesControlTargetPos = Math.max(targetPositionWhenExtended, Math.max(extendedMinPos, startFlipThresh));
                intakeFlipServo.setTargetAngle(Math.toRadians(flipAngleToGoOverBarrier * flipGearRatio), 1.0);
                if (this.isExtensionAtTarget()) this.intakeState = IntakeState.DROP_DOWN;
                else break;
            case DROP_DOWN:
                setRollerOff();
                slidesControlTargetPos = Math.max(targetPositionWhenExtended, Math.max(extendedMinPos, startFlipThresh));
                intakeFlipServo.setTargetAngle(Math.toRadians(flipDownAngle * flipGearRatio), 1.0);
                if (intakeFlipServo.inPosition()) {
                    intakeState = IntakeState.EXTENDED;
                    sampleCheckTime = currentTime;
                    didStopRoller = false;
                    setRollerOn();
                } else break;
            case EXTENDED:
                slidesControlTargetPos = targetPositionWhenExtended;
                intakeFlipServo.setTargetAngle(Math.toRadians(flipDownAngle * flipGearRatio), 1.0);
                if (sampleColor == Sensors.BlockColor.NONE) {
                    didStopRoller = false;
                    sampleCheckTime = currentTime;
                } else if (currentTime > sampleCheckTime + sampleConfirmDuration * 1e6) {
                    if (Globals.isRed ? sampleColor == Sensors.BlockColor.BLUE : this.sampleColor == Sensors.BlockColor.RED) {
                        if (autoUnjamIntake) setRollerUnjam();
                        else if (!didStopRoller) {
                            setRollerOff();
                            didStopRoller = true;
                        }
                    } else {
                        if (!didStopRoller) {
                            setRollerOff();
                            didStopRoller = true;
                        }
                        if (autoRetractIntake) intakeState = IntakeState.PICK_UP;
                    }
                }
                if (intakeState != IntakeState.PICK_UP) break;
            case PICK_UP:
                setRollerKeepIn();
                slidesControlTargetPos = this.targetPositionWhenExtended;
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (Math.toDegrees(intakeFlipServo.getCurrentAngle()) / flipGearRatio <= flipAngleToGoOverBarrier) intakeState = IntakeState.RETRACTING;
                else break;
            case RETRACTING:
                setRollerKeepIn();
                slidesControlTargetPos = startFlipThresh;
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (intakeFlipServo.inPosition()) {
                    intakeState = IntakeState.FINISH_RETRACTING;
                    lastRetractTime = currentTime;
                } else break;
            case FINISH_RETRACTING:
                setRollerKeepIn();
                slidesControlTargetPos = 0;
                intakeFlipServo.setTargetAngle(0, 1.0);
                if (isExtensionAtTarget()) intakeState = IntakeState.IDLE;
                else if (currentTime > lastRetractTime + retractMaxDuration * 1e6) setSlidesZero();
                else break;
            case IDLE:
                setRollerOff();
                slidesControlTargetPos = 0;
                intakeFlipServo.setTargetAngle(0, 1.0);
                break;
            case TRANSFER:
                setRollerSlowReverse();
                slidesControlTargetPos = 0;
                intakeFlipServo.setTargetAngle(0, 1.0);
                break;
        }

        // Roller control
//        switch (this.intakeRollerState) {
//            case OFF:
//                intakeRollerMotor.setTargetPower(0.0);
//                break;
//            case ON:
//                intakeRollerMotor.setTargetPower(1.0);
//                break;
//            case KEEP_IN:
//                intakeRollerMotor.setTargetPower(keepBlockInPower);
//                break;
//            case UNJAM:
//                intakeRollerMotor.setTargetPower(-1.0);
//                if (currentTime > this.unjamLastTime + unjamDuration * 1e6) setRollerOn();
//                break;
//            case REVERSE:
//                intakeRollerMotor.setTargetPower(-1.0);
//                break;
//            case SLOW_REVERSE:
//                intakeRollerMotor.setTargetPower(-slowReversePower);
//                break;
//        }

        // Extension control
        if (isExtensionAtTarget()) {
            pid.update(0,-1.0,1.0);
            pid.resetIntegral();
            intakeExtensionMotor.setTargetPower(0.0);
        } else {
            intakeExtensionMotor.setTargetPower(pid.update(slidesControlTargetPos - slidesCurrentPos, -1.0, 1.0));
        }

        this.updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake.intakeState", intakeState.toString());
        TelemetryUtil.packet.put("Intake.targetPositionWhenExtended", targetPositionWhenExtended);
        TelemetryUtil.packet.put("Intake.slidesControlTargetPos", slidesControlTargetPos);
        TelemetryUtil.packet.put("Intake.slidesCurrentPos", slidesCurrentPos);
        TelemetryUtil.packet.put("Intake::slidesBasePos", slidesBasePos);
    }

    /**
     * Gets the roller's state. -- Daniel
     * @return the roller's state (ON, OFF, KEEP_IN, UNJAM, REVERSE)
     */
    public IntakeRollerState getIntakeRollerState() { return intakeRollerState; }

    /**
     * Sets the roller to off. -- Daniel
     */
    public void setRollerOff() { intakeRollerState = IntakeRollerState.OFF; }

    /**
     * Sets the roller to on. -- Daniel
     */
    public void setRollerOn() { intakeRollerState = IntakeRollerState.ON; }

    /**
     * Sets the roller to keep the block in. -- Daniel
     */
    public void setRollerKeepIn() { intakeRollerState = IntakeRollerState.KEEP_IN; }

    /**
     * Sets the roller to unjam. -- Daniel
     */
    public void setRollerUnjam() { intakeRollerState = IntakeRollerState.UNJAM; unjamLastTime = System.nanoTime(); }

    /**
     * Sets the roller to reverse. -- Daniel
     */
    public void setRollerReverse() { intakeRollerState = IntakeRollerState.REVERSE; }

    /**
     * Sets the roller to slowly outtake the block. -- Daniel
     */
    public void setRollerSlowReverse() { intakeRollerState = IntakeRollerState.SLOW_REVERSE; }

    /**
     * Checks if the extension is at its target position. -- Daniel
     * @return whether the extension is at its target position
     */
    public boolean isExtensionAtTarget() { return Math.abs(slidesControlTargetPos - slidesCurrentPos) <= slidesTolerance; }

    /**
     * Gets the position the extension will go to when extended. -- Daniel
     * @return the extension target position, in inches
     */
    public double getTargetPositionWhenExtended() { return targetPositionWhenExtended; }

    /**
     * Sets the position the extension will go to when extended. The position is automatically clamped to range. -- Daniel
     * @param targetPositionWhenExtended the new extension target position, in inches
     */
    public void setTargetPositionWhenExtended(double targetPositionWhenExtended) { this.targetPositionWhenExtended = Utils.minMaxClip(targetPositionWhenExtended, extendedMinPos, slidesMaxPos); }

    /**
     * Gets the angle the actuation will go down to. -- Daniel
     * @return the actuation angle, in degrees
     */
    public double getFlipDownAngle() { return flipDownAngle; }

    /**
     * Sets the angle the actuation will go down to. The position is automatically clamped to range. -- Daniel
     * @param flipDownAngle the new actuation angle, in degrees
     */
    public void setFlipDownAngle(double flipDownAngle) { this.flipDownAngle = Utils.minMaxClip(flipDownAngle, flipDownAngleMin, flipDownAngleMax); }

    /**
     * Gets the intake's state. -- Daniel
     * @return the intake's state (START_EXTENDING, EXTENDING, EXTENDED, PICK_UP, RETRACTING, FINISH_RETRACTING, RETRACTED, IDLE, TRANSFER)
     */
    public IntakeState getIntakeState() { return intakeState; }

    /**
     * Sets the state to begin extending. -- Daniel
     */
    public void extend() {
        if (intakeState == IntakeState.IDLE) intakeState = IntakeState.START_EXTENDING;
        else if (intakeState == IntakeState.RETRACTING) intakeState = IntakeState.EXTENDING;
        else if (intakeState == IntakeState.PICK_UP) intakeState = IntakeState.DROP_DOWN;
    }

    /**
     * Sets the state to begin retracting, or exit transfer. -- Daniel
     */
    public void retract() {
        if (intakeState == IntakeState.EXTENDED || intakeState == IntakeState.DROP_DOWN) intakeState = IntakeState.PICK_UP;
        else if (intakeState == IntakeState.EXTENDING) intakeState = IntakeState.RETRACTING;
        else if (intakeState == IntakeState.TRANSFER) intakeState = IntakeState.IDLE;
    }

    /**
     * Sets the state to transfer. -- Daniel
     */
    public void transfer() {
        if (this.intakeState == IntakeState.IDLE) intakeState = IntakeState.TRANSFER;
    }

    /**
     * Sets the slides zero position. -- Daniel
     */
    public void setSlidesZero() {
        slidesBasePos = robot.sensors.getExtendoPosition() - slidesTolerance;
    }

    /**
     * Resets the slides zero position to the original value. -- Daniel
     */
    public void unsetSlidesZero() {
        slidesBasePos = 0;
    }

    /**
     * Checks if the intake is retracted. -- Daniel
     * @return whether the intake is retracted
     */
    public boolean isRetracted() { return intakeState == IntakeState.IDLE; }

    /**
     * Checks if the intake has a sample in it. -- Daniel
     * @return whether the intake has a sample in it
     */
    public boolean hasSample() { return this.sampleColor != Sensors.BlockColor.NONE; }
}

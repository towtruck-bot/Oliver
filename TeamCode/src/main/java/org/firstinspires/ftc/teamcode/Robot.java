package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.OldDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.ClawIntake;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final Sensors sensors;
    public final OldDrivetrain drivetrain;
    public final ClawIntake clawIntake;
    // public final Intake intake;
    public final Deposit deposit;
    public final Hang hang;

    public enum RobotState {
        RESET,
        IDLE,
        INTAKE_SAMPLE,
        TRANSFER,
        SAMPLE_READY,
        DEPOSIT_BUCKET,
        OUTTAKE,
        GRAB_SPECIMEN,
        SPECIMEN_READY,
        DEPOSIT_SPECIMEN
    }
    public enum NextState {
        DONE,
        INTAKE_SAMPLE,
        DEPOSIT,
        GRAB_SPECIMEN
    }
    private RobotState state = RobotState.IDLE;
    private RobotState prevState = RobotState.RESET;
    private RobotState prevState1 = RobotState.RESET;
    private NextState nextState = NextState.DONE;
    private long lastClickTime = -1;
    public static long bufferClickDuration = 500;

    private Func abortChecker;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Vision vision) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        if (Globals.hasSpecimenPreload) {
            this.state = RobotState.SPECIMEN_READY;
            this.prevState1 = RobotState.SPECIMEN_READY;
            this.prevState = RobotState.SPECIMEN_READY;
        } else if (Globals.hasSamplePreload) {
            this.state = RobotState.SAMPLE_READY;
            this.prevState1 = RobotState.SAMPLE_READY;
            this.prevState = RobotState.SAMPLE_READY;
        }

        sensors = new Sensors(this);
        clawIntake = new ClawIntake(this);
        // intake = new Intake(this);
        drivetrain = new OldDrivetrain(this);
        deposit = new Deposit(this);
        hang = new Hang(this);

        TelemetryUtil.setup();
        LogUtil.reset();
    }

    public void update() {
        START_LOOP();
        this.updateSubsystems();
        this.updateTelemetry();
    }

    private void updateSubsystems() {
        this.sensors.update();

        this.clawIntake.update();
        // intake.update();
        this.drivetrain.update();
        this.deposit.update();
        this.hang.update();

        this.robotFSM();

        this.hardwareQueue.update();
    }

/* Main Robot FSM diagram: "robot_fsm_v2.png" https://drive.google.com/drive/folders/1sDZOtl4i8u25d1JrAI3fPGEy5iU4Kujq?usp=sharing

    Single arrow: auto-advance state
    Double arrow: manual advance during Teleop
    Curly braces: these states have additional Teleop control (such as adjusting robot or claw position before continuing)

    v-----------------------------------------------------------------------------<
    V                                                                             |
    IDLE <<>> {INTAKE_SAMPLE} <<>> SAMPLE_READY >> TRANSFER > {DEPOSIT_BUCKET} >>-^
    ^  VV                                             V                           |
    |  >>--------------->> {GRAB_SPECIMEN}         OUTTAKE >----------------------^
    ^--------------------------<< vv ^^              ^^
    |                             vv ^^              ^^
    ^-<< {DEPOSIT_SPECIMEN} << SPECIMEN_READY >>-----^^
*/

    private void robotFSM() {
        long currentTime = System.nanoTime();
        boolean wasClicked = this.lastClickTime != -1 && currentTime - this.lastClickTime <= bufferClickDuration * 1e6;

        switch (this.state) {
            case IDLE:
                if (this.prevState != RobotState.IDLE) this.deposit.retract();
                if (wasClicked) {
                    if (this.nextState == NextState.INTAKE_SAMPLE) this.state = RobotState.INTAKE_SAMPLE;
                    else if (this.nextState == NextState.GRAB_SPECIMEN) this.state = RobotState.GRAB_SPECIMEN;
                    this.lastClickTime = -1;
                }
                break;
            case INTAKE_SAMPLE:
                if (this.prevState != RobotState.INTAKE_SAMPLE) {
                    this.clawIntake.extend();
                    this.deposit.prepareTransfer();
                }
                if (this.clawIntake.isRetracted()) {
                    if (this.clawIntake.hasSample()) this.state = RobotState.SAMPLE_READY;
                    else this.state = RobotState.IDLE;
                } else if (wasClicked && this.nextState == NextState.DONE) {
                    this.clawIntake.retract();
                    this.lastClickTime = -1;
                }
                break;
            case SAMPLE_READY:
                if (wasClicked) {
                    if (this.nextState == NextState.INTAKE_SAMPLE) state = RobotState.INTAKE_SAMPLE;
                    else if (this.nextState == NextState.DEPOSIT) state = RobotState.TRANSFER;
                    else if (this.nextState == NextState.DONE) state = RobotState.TRANSFER;
                    lastClickTime = -1;
                }
                break;
            case TRANSFER:
                if (this.prevState != RobotState.TRANSFER) this.deposit.startTransfer();
                if (this.deposit.isSampleReady()) this.state = this.nextState == NextState.DONE ? RobotState.OUTTAKE : RobotState.DEPOSIT_BUCKET;
                break;
            case DEPOSIT_BUCKET:
                if (prevState != RobotState.DEPOSIT_BUCKET) deposit.startSampleDeposit();
                if (deposit.isSampleDepositDone()) state = RobotState.IDLE;
                else if (wasClicked && this.nextState == NextState.DONE) {
                    deposit.finishSampleDeposit();
                    lastClickTime = -1;
                }
                break;
            case OUTTAKE:
                if (this.prevState != RobotState.OUTTAKE) this.deposit.startOuttake();
                if (this.deposit.isOuttakeDone()) this.state = RobotState.IDLE;
                break;
            case GRAB_SPECIMEN:
                if (this.prevState != RobotState.GRAB_SPECIMEN) this.deposit.startSpecimenGrab();
                if (this.deposit.isSpecimenReady()) this.state = RobotState.SPECIMEN_READY;
                else if (wasClicked) {
                    if (this.nextState == NextState.DONE) this.state = RobotState.IDLE;
                    else if (this.nextState == NextState.GRAB_SPECIMEN) this.deposit.finishSpecimenGrab();
                    this.lastClickTime = -1;
                }
                break;
            case SPECIMEN_READY:
                if (wasClicked) {
                    if (this.nextState == NextState.DONE) this.state = RobotState.OUTTAKE;
                    else if (this.nextState == NextState.DEPOSIT) this.state = RobotState.DEPOSIT_SPECIMEN;
                    else if (this.nextState == NextState.GRAB_SPECIMEN) this.state = RobotState.GRAB_SPECIMEN;
                    this.lastClickTime = -1;
                }
                break;
            case DEPOSIT_SPECIMEN:
                if (this.prevState != RobotState.DEPOSIT_SPECIMEN) this.deposit.startSpecimenDeposit();
                if (this.deposit.isSpecimenDepositDone()) this.state = RobotState.IDLE;
                else if ((wasClicked) && this.nextState == NextState.DONE) {
                    this.deposit.finishSpecimenDeposit();
                    this.lastClickTime = -1;
                }
                break;
        }
        prevState = prevState1;
        prevState1 = state;
    }

    /**
     * Gets the Robot FSM's state. -- Daniel
     * @return the Robot FSM's state
     */
    public RobotState getState() { return this.state; }

    /**
     * Sets what the robot will do next. Use in Teleop. -- Daniel
     *  <table>
     *      <tr>
     *          <td>Request</td>
     *          <td>Robot state</td>
     *          <td>Result</td>
     *      </tr>
     *      <tr>
     *          <td>DONE</td>
     *          <td>DEPOSIT_BUCKET, DEPOSIT_SPECIMEN</td>
     *          <td>Finish the deposit</td>
     *      </tr>
     *      <tr>
     *          <td>DONE</td>
     *          <td>INTAKE SAMPLE, GRAB SPECIMEN</td>
     *          <td>Retract the intake/grab</td>
     *      </tr>
     *      <tr>
     *          <td>DONE</td>
     *          <td>SAMPLE_READY, SPECIMEN_READY</td>
     *          <td>Drop it out (on deposit side)</td>
     *      </tr>
     *      <tr>
     *          <td>INTAKE_SAMPLE</td>
     *          <td>IDLE, SAMPLE_READY</td>
     *          <td>Extend the intake to get a sample</td>
     *      </tr>
     *      <tr>
     *          <td>DEPOSIT</td>
     *          <td>SAMPLE_READY</td>
     *          <td>Deposit it in the bucket</td>
     *      </tr>
     *      <tr>
     *          <td>DEPOSIT</td>
     *          <td>SPECIMEN_READY</td>
     *          <td>Hang it on the rod</td>
     *      </tr>
     *      <tr>
     *          <td>GRAB_SPECIMEN</td>
     *          <td>IDLE, SPECIMEN_READY</td>
     *          <td>Bring the claw down to grab a specimen</td>
     *      </tr>
     *      <tr>
     *          <td>GRAB_SPECIMEN</td>
     *          <td>GRAB_SPECIMEN</td>
     *          <td>Grab the specimen</td>
     *      </tr>
     *  </table>
     * An action can be requested slightly earlier than when the robot is ready to do it (buffer-click).
     * @param nextState what the robot will do next
     */
    public void setNextState(NextState nextState) {
        Log.i("FSM", this.state + ", " + nextState);
        this.nextState = nextState;
        this.lastClickTime = System.nanoTime();
    }

    /**
     * Restarts the current state. -- Daniel
     */
    public void restartState() {
        this.prevState = this.prevState1 = RobotState.RESET;
        this.deposit.holdSlides = false;
    }

    public void updateDepositHeights(boolean speciMode, boolean high){
        if (speciMode) {
            if (high) this.deposit.setDepositHighSpeci();
            else this.deposit.setDepositLowSpeci();
        } else {
            if (high) this.deposit.setDepositHeightHighSample();
            else this.deposit.setDepositHeightLowSample();
        }
    }

    /**
     * Sets the condition that should abort waiting (waitWhile, goToPoint)
     * @param func the function to check (return false to abort)
     */
    public void setAbortChecker(Func func) { this.abortChecker = func; }

    /**
     * Waits while a condition is true
     * @param func the function to check
     */
    public void waitWhile(Func func) {
        do {
            update();
        } while (((boolean) this.abortChecker.call()) && ((boolean) func.call()));
    }

    /**
     * Waits for a duration
     * @param duration the duration in milliseconds
     */
    public void waitFor(long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (((boolean) this.abortChecker.call()) && System.currentTimeMillis() - start < duration);
    }

//    public void goToPoint(Pose2d pose, Func func, boolean moveNear, boolean slowDown, boolean stop, double maxPower) {
//        long start = System.currentTimeMillis();
//        drivetrain.goToPoint(pose, moveNear, slowDown, stop, maxPower); // need this to start the process so thresholds don't immediately become true
//        do {
//            update();
//        } while (((boolean) this.abortChecker.call()) && (func == null || (boolean) func.call()) && System.currentTimeMillis() - start <= 5000 && drivetrain.isBusy());
//    }

    public void goToPoint(Pose2d pose, Func func, boolean finalAdjustment, boolean stop, double maxPower) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, maxPower); // need this to start the process so thresholds don't immediately become true
        do {
            update();
        } while (((boolean) this.abortChecker.call()) && (func == null || (boolean) func.call()) && System.currentTimeMillis() - start <= 5000 && drivetrain.isBusy());
    }

    public void goToPointWithIntake(Pose2d pose, Func func, boolean moveNear, boolean slowDown, boolean stop, double maxPower, boolean grab) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, moveNear, slowDown, stop, maxPower, grab);
        setNextState(NextState.INTAKE_SAMPLE);
        clawIntake.setIntakeTargetPos(12.0);

        do {
            update();
            clawIntake.grab(!grab);
            if (drivetrain.state != OldDrivetrain.State.GO_TO_POINT) { /* Should use drivetrain.state == Drivetrain.DriveState.WAIT_AT_POINT*/
                clawIntake.setIntakeTargetPos(drivetrain.getExtension());
                clawIntake.setClawRotation(Utils.headingClip(pose.heading - sensors.getOdometryPosition().heading));
                //TelemetryUtil.packet.put("auto aim", drivetrain.getExtension());

                if (clawIntake.isExtensionAtTarget() && drivetrain.state == OldDrivetrain.State.WAIT_AT_POINT) {
                    clawIntake.grab(grab);
                }
            } else if (Math.abs(drivetrain.getTurnError()) < Math.toRadians(30)) {
                //TelemetryUtil.packet.put("auto aim", -1);
                clawIntake.setIntakeTargetPos(12.0);
            }
        } while (((boolean) this.abortChecker.call()) && (func == null || (boolean) func.call()) && System.currentTimeMillis() - start <= 5000 && !(
            grab ? clawIntake.hasSample() && clawIntake.grabFinished() : !clawIntake.hasSample() && clawIntake.dropFinished()
        ));

        clawIntake.grab(grab);
        setNextState(NextState.DONE);
    }

    public void followSpline(Spline spline, Func func) {
        long start = System.currentTimeMillis();
        drivetrain.setPath(spline);
        drivetrain.state = OldDrivetrain.State.GO_TO_POINT;
        drivetrain.setMaxPower(1);
        update();

        do {
            update();
        } while (((boolean) this.abortChecker.call()) && (func == null || (boolean) func.call()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

    public void setIntakeExtension(double target){
        clawIntake.setIntakeTargetPos(target);
    }

    public void grab(boolean closed){
        clawIntake.grab(closed);
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Robot.state", this.state.toString());
        LogUtil.robotState.set(this.state.toString());
        //TelemetryUtil.packet.put("Robot.prevState1", this.prevState1.toString());
        //TelemetryUtil.packet.put("Robot.prevState", this.prevState.toString());
        TelemetryUtil.packet.put("Robot.nextState", this.nextState.toString());
        //TelemetryUtil.packet.put("Globals::RUNMODE", Globals.RUNMODE);
        //TelemetryUtil.packet.put("Globals::TESTING_DISABLE_CONTROL", Globals.TESTING_DISABLE_CONTROL);
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        //LogUtil.loopTime.set(GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
        LogUtil.send();
    }
}

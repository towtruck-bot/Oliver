package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.intake.ClawIntake;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final ClawIntake clawIntake;
    public final Deposit deposit;

    public enum RobotState {
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
    private RobotState prevState = RobotState.OUTTAKE;
    private RobotState prevState1 = RobotState.IDLE;
    private NextState nextState = NextState.DONE;
    private long lastClickTime = -1;
    public static long bufferClickDuration = 100;

    private boolean outtakeAndThenGrab = false;

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
        this.sensors = new Sensors(this);
        this.clawIntake = new ClawIntake(this);
        this.drivetrain = new Drivetrain(this);
        this.deposit = new Deposit(this);

        TelemetryUtil.setup();
    }

    public void update() {
        START_LOOP();
        this.updateSubsystems();
        this.updateTelemetry();
    }

    private void updateSubsystems() {
        this.sensors.update();

        this.clawIntake.update();
        this.drivetrain.update();
        this.deposit.update();

        this.robotFSM();

        this.hardwareQueue.update();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Robot.state", this.state.toString());
        TelemetryUtil.packet.put("Robot.prevState1", this.prevState1.toString());
        TelemetryUtil.packet.put("Robot.prevState", this.prevState.toString());
        TelemetryUtil.packet.put("Robot.nextState", this.nextState.toString());
        TelemetryUtil.packet.put("Robot.outtakeAndThenGrab", this.outtakeAndThenGrab);
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
    }

    public void followSpline(Spline spline, Func func) {
        long start = System.currentTimeMillis();
        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.setMaxPower(1);
        update();

        do {
            update();
        } while (((boolean) func.call()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

/* Main Robot FSM diagram below

    Single arrow: auto-advance state
    Double arrow: manual advance during Teleop
    Curly braces: these states have additional Teleop control (such as adjusting robot or claw position before continuing)

    v--------------<---------------------------------------------------------<
    V              ^^                                                        |
    IDLE >> {INTAKE_SAMPLE} >> TRANSFER > SAMPLE_READY >> {DEPOSIT_BUCKET} >-^
    ^  VV                                      VV                            |
    |  >>--------------->> {GRAB_SPECIMEN} < OUTTAKE >-----------------------^
    |                          VV   VV          ^^
    ^--------------------------<    VV          ^^
    |                               VV          ^^
    ^-< {DEPOSIT_SPECIMEN} << SPECIMEN_READY >>-^^
*/

    private void robotFSM() {
        long currentTime = System.nanoTime();
        boolean wasClicked = this.lastClickTime != -1 && currentTime - this.lastClickTime <= bufferClickDuration * 1e6;

        switch (this.state) {
            case IDLE:
                if (this.prevState != RobotState.IDLE)
                    this.deposit.retract();
                if (wasClicked) {
                    if (this.nextState == NextState.INTAKE_SAMPLE)
                        this.state = RobotState.INTAKE_SAMPLE;
                    else if (this.nextState == NextState.GRAB_SPECIMEN)
                        this.state = RobotState.GRAB_SPECIMEN;
                    this.lastClickTime = -1;
                }
                break;
            case INTAKE_SAMPLE:
                if (this.prevState != RobotState.INTAKE_SAMPLE) {
                    this.clawIntake.extend();
                    this.deposit.prepareTransfer();
                }
                if (this.clawIntake.isRetracted()) {
                    if (this.clawIntake.hasSample())
                        this.state = RobotState.SAMPLE_READY;
                    else
                        this.state = RobotState.IDLE;
                } else if (wasClicked && this.nextState == NextState.DONE) {
                    this.clawIntake.retract();
                    this.lastClickTime = -1;
                }
                break;
            case SAMPLE_READY:
                if (wasClicked) {
                    if (nextState == NextState.INTAKE_SAMPLE)
                        state = RobotState.INTAKE_SAMPLE;
                    else if (nextState == NextState.DEPOSIT)
                        state = RobotState.TRANSFER;
                    else
                    lastClickTime = -1;
                }
                break;
            case TRANSFER:
                if (this.prevState != RobotState.TRANSFER)
                    this.deposit.startTransfer();
                if (this.deposit.isSampleReady())
                    this.state = RobotState.DEPOSIT_BUCKET;
                break;
            case DEPOSIT_BUCKET:
                if (prevState != RobotState.DEPOSIT_BUCKET)
                    deposit.startSampleDeposit();
                if (deposit.isSampleDepositDone())
                    state = RobotState.IDLE;
                else if (wasClicked && nextState == NextState.DONE) {
                    deposit.finishSampleDeposit();
                    lastClickTime = -1;
                }
                break;
            case OUTTAKE:
                if (this.prevState != RobotState.OUTTAKE)
                    this.deposit.startOuttake();
                if (this.deposit.isOuttakeDone()) {
                    if (this.outtakeAndThenGrab) { //TODO: i don tthink teleop ever set this my bad? unless robot does and i missed it
                        this.state = RobotState.GRAB_SPECIMEN;
                    } else {
                        this.state = RobotState.IDLE;
                        //this.outtakeAndThenGrab = true;
                    }
                }
                break;
            case GRAB_SPECIMEN:
                if (this.prevState != RobotState.GRAB_SPECIMEN)
                    this.deposit.startSpecimenGrab();
                if (this.deposit.isSpecimenReady())
                    this.state = RobotState.SPECIMEN_READY;
                else if (wasClicked) {
                    if (this.nextState == NextState.DONE)
                        this.state = RobotState.IDLE;
                    else if (this.nextState == NextState.GRAB_SPECIMEN)
                        this.deposit.finishSpecimenGrab();
                    this.lastClickTime = -1;
                }
                break;
            case SPECIMEN_READY:
                if (wasClicked) {
                    if (this.nextState == NextState.DONE)
                        this.state = RobotState.OUTTAKE;
                    else if (this.nextState == NextState.DEPOSIT)
                        this.state = RobotState.DEPOSIT_SPECIMEN;
                    this.lastClickTime = -1;
                }
                break;
            case DEPOSIT_SPECIMEN:
                if (this.prevState != RobotState.DEPOSIT_SPECIMEN)
                    this.deposit.startSpecimenDeposit();
                if (this.deposit.isSpecimenDepositDone())
                    this.state = RobotState.IDLE;
                else if (wasClicked && this.nextState == NextState.DONE) {
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

    public void forceRetractToSampleReady(){
        state = RobotState.SAMPLE_READY;
        prevState = RobotState.TRANSFER;
    }

    public void forceRetractToSpeciReady(){
        state = RobotState.SPECIMEN_READY;
        prevState = RobotState.GRAB_SPECIMEN;
    }
    /**
     * Sets what will happen after an OUTTAKE. This is automatically set to true after an OUTTAKE > IDLE. -- Daniel
     * @param outtakeAndThenGrab true if the robot should go directly to GRAB_SPECIMEN, false to go to IDLE
     */
    public void setOuttakeAndThenGrab(boolean outtakeAndThenGrab) { this.outtakeAndThenGrab = outtakeAndThenGrab; }

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
     *          <td>Cancel the intake/grab</td>
     *      </tr>
     *      <tr>
     *          <td>DONE</td>
     *          <td>SPECIMEN_READY</td>
     *          <td>Outtake it</td>
     *      </tr>
     *      <tr>
     *          <td>INTAKE_SAMPLE</td>
     *          <td>IDLE</td>
     *          <td>Extend the intake to get a sample</td>
     *      </tr>
     *      <tr>
     *          <td>INTAKE_SAMPLE</td>
     *          <td>SAMPLE_READY</td>
     *          <td>Extend the intake, dropping the current sample</td>
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
     *          <td>IDLE</td>
     *          <td>Bring the claw down to grab a specimen</td>
     *      </tr>
     *      <tr>
     *          <td>GRAB_SPECIMEN</td>
     *          <td>GRAB_SPECIMEN</td>
     *          <td>Close the claw</td>
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

    public void updateDepositHeights(boolean speciMode, boolean high){
        if(speciMode){
            if(high){
                this.deposit.setDepositHighSpeci();
            }else{
                this.deposit.setDepositLowSpeci();
            }
        }else{
            if(high){
                this.deposit.setDepositHeightHighSample();
            }else{
                this.deposit.setDepositHeightLowSample();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.nSensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeSoloLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.OneHundredMSIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;

public class nDrivetrain {
    public enum State{
        DRIVE,
        GO_TO_POINT,
        FOLLOW_SPLINE,
        FINAL_ADJUSTMENT,
        BRAKE,
        WAIT_AT_POINT,
        IDLE
    }
    public State state = State.IDLE;

    private Vision vision;
    private nSensors sensors;
    private Localizer localizers;

    public nPriorityMotor leftFront, leftRear, rightRear, rightFront;
    private List<nPriorityMotor> motors;

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = new double[] {
            0.3121803239920063,
            0.3533249418072871,
            0.36038420175052865,
            0.39695077434023707
    };

    public nDrivetrain (Robot robot){
        // this.sensors = robot.sensors
        HardwareMap hardwareMap = robot.hardwareMap;

        leftFront = new nPriorityMotor(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                "leftFront",
                4, 5, 1.0, sensors
        );

        leftRear = new nPriorityMotor(
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                "leftRear",
                4, 5, -1.0, sensors
        );
        rightRear = new nPriorityMotor(
                hardwareMap.get(DcMotorEx.class, "rightRear"),
                "rightRear",
                4, 5, sensors
        );
        rightFront = new nPriorityMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                "rightFront",
                4, 5, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (nPriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            robot.hardwareQueue.addDevice(motor);
        }

        // uncomment once finished
//        localizers = new Localizer[]{
//                new IMUMergeSoloLocalizer(hardwareMap, sensors, this, "#0000ff", "#ff00ff"),
//                new IMULocalizer(hardwareMap, sensors, this, "#ff0000", "#00ff00"),
//                new IMUMergeLocalizer(hardwareMap, sensors, this, "#ffff00", "#00ffff"),
//                new OneHundredMSIMULocalizer(hardwareMap, sensors, this, "#aa0000", "#00ee00"),
//                new TwoWheelLocalizer(hardwareMap, sensors, this, "#aaaa00", "#00aaaa"),
//                new Localizer(hardwareMap, sensors, this, "#0000aa", "#aa00aa")
//        };

        setMinPowersToOvercomeFriction();
    }

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]);
        for (nPriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
        }
    }
}

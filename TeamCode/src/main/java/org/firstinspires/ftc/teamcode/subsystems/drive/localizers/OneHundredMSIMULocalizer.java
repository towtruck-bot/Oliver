package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.OldDrivetrain;

public class OneHundredMSIMULocalizer extends Localizer {
    private long lastUpdate;

    public OneHundredMSIMULocalizer(HardwareMap hardwareMap, Sensors sensors, OldDrivetrain drivetrain, String color, String expectedColor) {
        super(hardwareMap, sensors, drivetrain, color, expectedColor);
        lastUpdate = System.currentTimeMillis();
    }

    @Override
    public void update() {
        super.update();
        if (System.currentTimeMillis() - lastUpdate > 100){
            heading = currentPose.heading = sensors.getHeading();
            lastUpdate = System.currentTimeMillis();
        }
    }
}

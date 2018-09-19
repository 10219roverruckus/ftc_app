package org.firstinspires.ftc.teamcode.outreach.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class outreachTouchSensorCatapult {

    private DigitalChannel catapultTouchSensor;

    public outreachTouchSensorCatapult (DigitalChannel cTS) {
        catapultTouchSensor = cTS;
        catapultTouchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean checkCatapultTouchSensor () {
        if (catapultTouchSensor.getState() == true) {
            return true;
        }
        else {
            return false;
        }
    }
}

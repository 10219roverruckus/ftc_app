package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;

import android.graphics.Color;


public class RevColorDistance {
    public ColorSensor revColorSensor;
    public DistanceSensor revDistanceSensor;

    // mineral lift sensors
    public ColorSensor revColorSensorMineralLift;
    public DistanceSensor revDistanceSensorMineralLift;

    // hook lift sensors
    public ColorSensor revColorSensorHookLift;
    public DistanceSensor revDistanceSensorHookLift;

    // extender arm sensors
    public ColorSensor revColorSensorExtender;
    public DistanceSensor revDistanceSensorExtender;


    MecanumDrive myMechDrive;


    // other instance variables

    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

//    public final int MINERAL_LIFT_THRESHOLD = 100;            //

    public final int MINERAL_LIFT_THRESHOLD_NOTHING = 0;       // Original was 180, adjusted for red hue
    public final int MINERAL_LIFT_THRESHOLD_RED = 18;          // Original was 270
    public final int HOOK_LIFT_THRESHOLD = 100;
    public final int EXTENDER_THRESHOLD = 100;

    final double SCALE_FACTOR = 255;

    public LinearOpMode linearOp = null;

    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    // constructers
    public RevColorDistance(ColorSensor rCS, DistanceSensor rDS) {
        revColorSensor = rCS;
        revDistanceSensor = rDS;

    }

    public boolean checkSensorMineralLift() {

        Color.RGBToHSV((int) (revColorSensor.red() * SCALE_FACTOR),     // Move backwards until color detected
                (int) (revColorSensor.green() * SCALE_FACTOR),
                (int) (revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);


        if (hsvValues[0] > MINERAL_LIFT_THRESHOLD_NOTHING && hsvValues[0] <  MINERAL_LIFT_THRESHOLD_RED) {
            return true;

        }
        else {
            return false;
        }

    }

    public boolean checkSensorHookLift () {
        Color.RGBToHSV((int) (revColorSensorHookLift.red() * SCALE_FACTOR),     // Move backwards until color detected
                (int) (revColorSensorHookLift.green() * SCALE_FACTOR),
                (int) (revColorSensorHookLift.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[0] > HOOK_LIFT_THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean checkSensorExtender () {
        Color.RGBToHSV((int) (revColorSensorExtender.red() * SCALE_FACTOR),     // Move backwards until color detected
                (int) (revColorSensorExtender.green() * SCALE_FACTOR),
                (int) (revColorSensorExtender.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[0] > EXTENDER_THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

}

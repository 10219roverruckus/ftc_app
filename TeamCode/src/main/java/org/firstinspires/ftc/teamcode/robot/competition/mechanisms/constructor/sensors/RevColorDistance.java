package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class RevColorDistance {
    public ColorSensor revColorSensor;
    public DistanceSensor revDistanceSensor;

    public LinearOpMode linearOp = null;
    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public RevColorDistance (ColorSensor rCS, DistanceSensor rDS) {
        revColorSensor = rCS;
        revDistanceSensor = rDS;
    }
}

package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//import org.opencv.core.Range;

@TeleOp(name = "TEST - TEAM MARKER SERVO")

public class testTeamMarkerServo extends OpMode {
    Servo teamMarkerArm;

    final double SERVO_LOW = 0;
    final double SERVO_MID = 0.5;
    final double SERVO_HIGH = 1.0;

    double teamMarkerPosition;

    @Override
    public void init() {
        teamMarkerPosition = SERVO_MID;
        teamMarkerArm = hardwareMap.servo.get ("team_marker_arm");
        teamMarkerArm.setPosition(teamMarkerPosition);
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            teamMarkerPosition = teamMarkerPosition + .001;
        }
        if (gamepad2.a) {
            teamMarkerPosition = teamMarkerPosition - .001;
        }
        if (gamepad2.x || gamepad2.b) {
            teamMarkerPosition = SERVO_MID;
        }

        teamMarkerPosition = Range.clip(teamMarkerPosition,SERVO_LOW,SERVO_HIGH);
        teamMarkerArm.setPosition(teamMarkerPosition);
        telemetry.addData("servo position: ", teamMarkerArm.getPosition());
    }
}

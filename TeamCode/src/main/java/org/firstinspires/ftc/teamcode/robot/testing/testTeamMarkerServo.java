package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.opencv.core.Range;

@TeleOp(name = "TEST - SERVO DUMPER")
//@Disabled
public class testTeamMarkerServo extends OpMode {
    Servo servotest;

    final double SERVO_LOW = .56;
//    final double SERVO_MID = 0.5;
    final double SERVO_HIGH = 1.0;

    public double LSScore = .56;
    public double LSCollect = 1.0;


    double servoPosition;
    public ElapsedTime armRunTime;

    @Override
    public void init() {
        servotest = hardwareMap.servo.get ("mineral_dumper");
//        servotest.setPosition(servoPosition);
//        servotest.setPosition(SERVO_HIGH);
        armRunTime = new ElapsedTime();
        armRunTime.reset();
    }

    @Override
    public void loop() {
        if (gamepad2.y) {
            servoPosition = servoPosition + .00025;
        }
        if (gamepad2.a) {
            servoPosition = servoPosition - .00025;
        }
        if (gamepad2.dpad_up) {
            servoPosition = SERVO_HIGH;
        }
        if (gamepad2.dpad_down) {
            servoPosition = SERVO_LOW;
        }
        armRunTime.time();
        servoPosition = Range.clip(servoPosition, 0, 1);
        servotest.setPosition(servoPosition);
        telemetry.addData("servo position! ", servotest.getPosition());
        telemetry.addData("time! ", armRunTime.time());
    }
}

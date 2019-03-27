package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.opencv.core.Range;

@TeleOp(name = "Calibrate TWO Servos BOTTOM- P2 y+a", group = "CALIBRATION")
//@Disabled
public class ServoTWOCalibrationBOTTOM extends OpMode {
    Servo servotest_TOP;
    Servo servotest_BOTTOM;

    final double INCREMENTLEVEL = .00025;

    final double SERVO_LOW = 0.9; //.99
    final double SERVO_MID = 0.8;
    final double SERVO_HIGH = 1.0;

    public double LSScore = .56;        //.56
    public double LSCollect = 1.0;      //1.0


    double servoPosition_TOP;
    double servoPosition_BOTTOM = SERVO_LOW;
    public ElapsedTime armRunTime;

    @Override
    public void init() {
        servotest_TOP = hardwareMap.servo.get ("rotator_top");
        servotest_BOTTOM = hardwareMap.servo.get ("rotator_bottom");
//        servotest.setPosition(servoPosition);
//        servotest.setPosition(SERVO_HIGH);
        armRunTime = new ElapsedTime();
        armRunTime.reset();
//        servotest_TOP.setPosition(SERVO_MID);
        servotest_BOTTOM.setPosition(SERVO_LOW);
    }

    @Override
    public void loop() {
        if (gamepad2.b) {
//            servotest_TOP.setPosition(SERVO_MID);
            servotest_BOTTOM.setPosition(SERVO_MID);
            armRunTime.reset();
        }
        if (gamepad2.y) {
//            servoPosition_TOP = servoPosition_TOP + INCREMENTLEVEL;
            servoPosition_BOTTOM = servoPosition_BOTTOM + INCREMENTLEVEL;
        }
        if (gamepad2.a) {
//            servoPosition_TOP = servoPosition_TOP - INCREMENTLEVEL;
            servoPosition_BOTTOM = servoPosition_BOTTOM - INCREMENTLEVEL;
        }
        if (gamepad2.dpad_up) {
//            servoPosition = SERVO_HIGH;
        }
        if (gamepad2.dpad_down) {
//            servoPosition = SERVO_LOW;
        }
        armRunTime.time();
//        servoPosition_TOP = Range.clip(servoPosition_TOP, 0, 1);
//        servotest_TOP.setPosition(servoPosition_TOP);
        servoPosition_BOTTOM = Range.clip(servoPosition_BOTTOM, 0, 1);
        servotest_BOTTOM.setPosition(servoPosition_BOTTOM);
        telemetry.addLine("y = up; a = down; b = RESET");
        telemetry.addData("servo TOP position! ", servotest_TOP.getPosition());
        telemetry.addData("servo BOTTOM position! ", servotest_BOTTOM.getPosition());
        telemetry.addData("time! ", armRunTime.time());
    }
}

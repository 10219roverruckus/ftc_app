package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.opencv.core.Range;

@TeleOp(name = "Calibrate TWO Servos- P2 y+a - B = LOW, X = HIGH", group = "CALIBRATION")
//@Disabled
public class ServoTWOCalibration extends OpMode {
    Servo servotest_TOP;
    Servo servotest_BOTTOM;

//    final double INCREMENTLEVEL = .00025;
    final double INCREMENTLEVEL = .001;

    final double SERVO_LOW = 0.9;
    final double SERVO_MID = 0.5;
    final double SERVO_HIGH = 0.23;

    public double LSScore = .56;        //.56
    public double LSCollect = 1.0;      //1.0


    double servoPosition_TOP = SERVO_HIGH;
    double servoPosition_BOTTOM = SERVO_HIGH;
    public ElapsedTime armRunTime;

    @Override
    public void init() {
        servotest_TOP = hardwareMap.servo.get ("rotator_top");
        servotest_BOTTOM = hardwareMap.servo.get ("rotator_bottom");
//        servotest.setPosition(servoPosition);
//        servotest.setPosition(SERVO_HIGH);
        armRunTime = new ElapsedTime();
        armRunTime.reset();
        servotest_TOP.setPosition(servoPosition_TOP);
        servotest_BOTTOM.setPosition(servoPosition_BOTTOM);
    }

    @Override
    public void loop() {
        if (gamepad2.b) {
            servoPosition_TOP = SERVO_LOW;
            servoPosition_BOTTOM = SERVO_LOW;
            armRunTime.reset();
        }

        if (gamepad2.x) {
            servoPosition_TOP = SERVO_HIGH;
            servoPosition_BOTTOM = SERVO_HIGH;
            armRunTime.reset();
        }

        if (gamepad2.y) {
            servoPosition_TOP = servoPosition_TOP + INCREMENTLEVEL;
            servoPosition_BOTTOM = servoPosition_BOTTOM + INCREMENTLEVEL;
        }
        if (gamepad2.a) {
            servoPosition_TOP = servoPosition_TOP - INCREMENTLEVEL;
            servoPosition_BOTTOM = servoPosition_BOTTOM - INCREMENTLEVEL;
        }
        if (gamepad2.dpad_up) {
//            servoPosition = SERVO_HIGH;
        }
        if (gamepad2.dpad_down) {
//            servoPosition = SERVO_LOW;
        }
        armRunTime.time();
        servoPosition_TOP = Range.clip(servoPosition_TOP, 0, 1);
        servotest_TOP.setPosition(servoPosition_TOP);
        servoPosition_BOTTOM = Range.clip(servoPosition_BOTTOM, 0, 1);
        servotest_BOTTOM.setPosition(servoPosition_BOTTOM);
        telemetry.addLine("y = up; a = down; b = RESET");
        telemetry.addData("servo TOP position! ", servotest_TOP.getPosition());
        telemetry.addData("servo BOTTOM position! ", servotest_BOTTOM.getPosition());
        telemetry.addData("time! ", armRunTime.time());
    }
}

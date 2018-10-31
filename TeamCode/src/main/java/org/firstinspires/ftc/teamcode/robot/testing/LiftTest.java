package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST LIFT ONLY")
public class LiftTest extends OpMode {
    DcMotor liftArmMotor;
    double rightJoystick_lift;
    @Override
    public void init() {
        liftArmMotor = hardwareMap.dcMotor.get("lift_motor");
    }

    @Override
    public void loop() {
        rightJoystick_lift = gamepad2.right_stick_y;    //assigns lift to right stick y

        if (rightJoystick_lift < -.1 || rightJoystick_lift > .1) {
            liftArmMotor.setPower(rightJoystick_lift);
        }
        else {
            liftArmMotor.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST LIFT - P2 R STICK - B RESET")
public class EncoderMotorCalibration extends OpMode {
    DcMotor liftArmMotor;  //NAME DOESN'T MATTER - BUT BE SURE TO CHANGE HARDWARE MAP AS NEEDED
    double rightJoystick_lift;
    @Override
    public void init() {
        liftArmMotor = hardwareMap.dcMotor.get("lift_motor");  //MAP TO YOUR RC CONFIG
//        liftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // REVERSE AS NEEDED
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        telemetry.addData("LIFT MOTOR POSITION: ", liftArmMotor.getCurrentPosition());
        if (gamepad2.b) {
            liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TEST MOTOR ENCODER - P2 R STICK - B RESET")
public class EncoderMotorCalibration extends OpMode {

    DcMotor myEncoderMotor;  //NAME DOESN'T MATTER - BUT BE SURE TO CHANGE HARDWARE MAP AS NEEDED
    double rightJoystick_gp2;
    double minMotorThreshold = 0.1; // min motor power so don't burn out motor. Also allows for joysticks to not stall at very low powers if resting position is something like .001

    @Override
    public void init() {
        myEncoderMotor = hardwareMap.dcMotor.get("mineral_lift_motor");  //MAP TO YOUR RC CONFIG
//        myEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // REVERSE AS NEEDED
        // Always STOP_AND_RESET_ENCODER BEFORE setting encoder mode (RUN_WITHOUT_ENCODER || RUN_USING_ENCODER || RUN_TO_POSITION).
        // Bad things await those swap the order...
        myEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        myEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myEncoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myEncoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        rightJoystick_gp2 = gamepad2.right_stick_y;    //assigns lift to right stick y GP2.  Don't need to assign a variable, but we choose to.
        if (rightJoystick_gp2 < -minMotorThreshold || rightJoystick_gp2 > minMotorThreshold) { //.1 = min motor threshold so don't burn out.
            myEncoderMotor.setPower(rightJoystick_gp2);
        }
        else {
            myEncoderMotor.setPower(0);
        }
        telemetry.addData("MOTOR POSITION: ", myEncoderMotor.getCurrentPosition());
        telemetry.addData("rightJoystick_gp2 VAR: ", rightJoystick_gp2);
        telemetry.addData("gamepad2.right_stick_y RAW: ", gamepad2.right_stick_y);
        if (gamepad2.b) {
            myEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myEncoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("RESETTING ENCODERS");
        }
    }
}
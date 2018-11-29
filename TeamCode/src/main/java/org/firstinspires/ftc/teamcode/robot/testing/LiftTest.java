package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TEST LIFT - P2 R STICK - B RESET")
public class LiftTest extends OpMode {
    DcMotor liftArmMotor;
    double rightJoystick_lift;
    Servo ledStrip;

    double minLedPosition = 0.2525;
    double maxLedPosition = 0.7475;
    double ledPosition = minLedPosition;
    double ledIncrementValue = 0.0001;

    //LED light instance variables
    double minLiftENC = 0;              // lowest value for the lifts hight
    double minLiftRange = 100;          // range that is for when the lights are going to be on
    double maxLiftENC = 1000;           // highest value for the lifts hight
    double maxLiftRange = 100;          // range that is for when the lights are going to be on

    public final double RED = .6674;
    public final double BLUE = .4942;
    public final double GREEN = .7112;
    public final double PURPLE = .6575;

    @Override
    public void init() {
        liftArmMotor = hardwareMap.dcMotor.get("lift_motor");
//        liftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ledStrip = hardwareMap.servo.get("led_strip");
        ledStrip.setPosition(ledPosition);
    }

    @Override
    public void loop() {

        double liftPosition =  liftArmMotor.getCurrentPosition();
        if (gamepad2.right_bumper) {
            ledPosition = ledPosition + ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }

        if (gamepad2.left_bumper) {
            ledPosition = ledPosition - ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }

        rightJoystick_lift = -gamepad2.right_stick_y;    //assigns lift to right stick y

        if (rightJoystick_lift < -.1 || rightJoystick_lift > .1) {
            telemetry.addLine("JOYSTICK IF");
            liftArmMotor.setPower(rightJoystick_lift);
        }
        else {
            telemetry.addLine("JOYSTICK 0");
            liftArmMotor.setPower(0);
        }

        if (ledPosition != ledStrip.getPosition()) {
            ledStrip.setPosition(ledPosition);
        }


        // LED LIFT LOGIC


        if (liftPosition >= minLiftENC && liftPosition <+ minLiftENC + minLiftRange) {           // setting the LED lights to blue
            // if the lift position is in between the range then the light will turn blue
        }
        if (liftPosition >= maxLiftENC - maxLiftRange && liftPosition <= maxLiftENC) {          // setting the LED lights to Red
            // if the lift position is in between the range then the light will turn Red
        }
        if (liftPosition >= minLiftRange && liftPosition <= maxLiftENC - maxLiftRange) {        // setting the LED lights to Green
            // if the lift position is in between the range then the light will turn Green
        }

        telemetry.addData("LED Pos VAR: ", ledPosition);
        telemetry.addData("LED getPosition RAW: ", ledStrip.getPosition());
        telemetry.addData("LIFT MOTOR POSITION: ", liftArmMotor.getCurrentPosition());
        telemetry.addData("Right Joystick Y VAR: ", rightJoystick_lift);
        telemetry.addData("Right Joystick Y RAW: ", gamepad2.right_stick_y);

        if (gamepad2.b) {
            liftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}

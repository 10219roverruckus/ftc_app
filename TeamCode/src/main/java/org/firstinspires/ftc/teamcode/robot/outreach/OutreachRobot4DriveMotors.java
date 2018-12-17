package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.internal.hardware.DragonboardLynxDragonboardIsPresentPin;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.CatapultArm;


import static java.lang.Thread.sleep;
//adb connect 1.2.3.4:5555

@TeleOp(name = "Outreach Robot - 4 Drive Motors")
//@Disabled
public class OutreachRobot4DriveMotors extends OpMode {

    OutreachMotors myOutreachMotors;

    outreachTouchSensorCatapult myOutreachTouchSensorCatapult;
    //value for left joystick
    double leftY;
    //value for right joystick
    double rightY;
    double leftX;
    double rightX;

    double triggerLeft;
    double triggerRight;
    DcMotor catapult;

    double minLedPosition = 0.2525;
    double maxLedPosition = 0.7475;
    double ledPosition = minLedPosition;
    double ledIncrementValue = 0.0002;

    Servo ledStrip;

    public DriveMode driveMode = null;
    public DriveDirection driveDirection = null;


    @Override
    public void init() {
        myOutreachMotors = new OutreachMotors(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        ledStrip = hardwareMap.servo.get("led_strip");
        ledStrip.setPosition(ledPosition);
        driveMode = driveMode.START;
        driveDirection = driveDirection.FORWARD;

    }

    @Override
    public void loop() {
        //assigns left and right joysticks
        gamePadAssignments();
        //checks for TANK / ARCADE / FORWARD / REVERSE drive modes.
        driveModeChecks();
        //drive the robot!!!!
        driveRobot();
        //control LEDs manually - use left & right bumpers
        controlLEDs();
        telemetryOutput();
    }

    public void driveRobot () {
        switch (driveMode) {
            case START:
                telemetry.addLine("Waiting for user to select DRIVE MODE");
                break;
            case TANK:
                myOutreachMotors.driveTank(gamepad1, driveDirection);
                break;
            case ARCADE:
                myOutreachMotors.arcadeDrive(gamepad1, driveDirection);
                break;
        }
    }

    public void driveModeChecks () {
        if (gamepad1.dpad_up){
            driveDirection = DriveDirection.FORWARD;
        }
        if (gamepad1.dpad_down) {
            driveDirection = DriveDirection.REVERSE;
        }
        if (gamepad1.dpad_right) {
//            driveMode = DriveMode.ARCADE;
        }
        if (gamepad1.dpad_left) {
            driveMode = DriveMode.TANK;
        }
    }

    public void gamePadAssignments () {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
    }

    public void controlLEDs () {
        if (gamepad1.left_bumper) {
            ledPosition = ledPosition - ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }
        if (gamepad1.right_bumper) {
            ledPosition = ledPosition + ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }
        if (gamepad1.left_stick_y > .1) {
            ledPosition = ledPosition - ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }
        if (gamepad1.left_stick_y < -.1) {
            ledPosition = ledPosition + ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }
        if (gamepad1.left_trigger == 1){
            ledPosition = minLedPosition;
        }
        if (gamepad1.right_trigger == 1) {
            ledPosition = maxLedPosition;
        }
//        if (ledPosition != ledStrip.getPosition()) {
//            ledStrip.setPosition(ledPosition);
//        }
        ledStrip.setPosition(ledPosition);
    }

    public void telemetryOutput () {
        telemetry.addData("LED Pos VAR: ", ledPosition);
        telemetry.addData("LED getPosition RAW: ", ledStrip.getPosition());
        telemetry.addData("DRIVE MODE ", driveMode);
        telemetry.addData("DRIVE DIRECTION: ", driveDirection);
        telemetry.addData("Left Y: ", gamepad1.left_stick_y);
        telemetry.addData("Right Y: ", gamepad1.right_stick_y);
        telemetry.addLine("D_PAD RIGHT FOR ARCADE MODE");
        telemetry.addLine("D_PAD LEFT FOR TANK DRIVE");
        telemetry.addLine("D_UP FOR FORWARD MODE");
        telemetry.addLine("D_DOWN FOR REVERSE MODE");
    }
}


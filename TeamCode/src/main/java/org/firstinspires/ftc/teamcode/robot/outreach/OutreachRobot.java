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

@TeleOp(name = "Outreach Robot")
//@Disabled
public class OutreachRobot extends OpMode {

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
    double ledIncrementValue = 0.0001;

    Servo ledStrip;

    public DriveMode driveMode = null;
    public DriveDirection driveDirection = null;
    boolean driveForward = true;

    boolean bumperAllow = true;

    @Override
    public void init() {
        myOutreachMotors = new OutreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        ledStrip = hardwareMap.servo.get("led_strip");
        ledStrip.setPosition(ledPosition);
        driveMode = driveMode.START;
        driveDirection = driveDirection.START;

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
                if (driveDirection == DriveDirection.FORWARD) {
                    myOutreachMotors.driveTank(-leftY, -rightY, true);
                }
                else if (driveDirection == DriveDirection.REVERSE) {
                    myOutreachMotors.driveTank(leftY, rightY, true);
                }
                break;
            case ARCADE:
                myOutreachMotors.arcadeDrive(gamepad1);
                break;
        }
    }

    public void driveModeChecks () {
        if (gamepad1.dpad_up){
            driveForward = true;
            driveDirection = DriveDirection.FORWARD;
        }
        if (gamepad1.dpad_down) {
            driveForward = false;
            driveDirection = DriveDirection.REVERSE;
        }
        if (gamepad1.dpad_right) {
            driveMode = DriveMode.ARCADE;
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
//        if (ledPosition != ledStrip.getPosition()) {
//            ledStrip.setPosition(ledPosition);
//        }
        ledStrip.setPosition(ledPosition);
    }

    public void telemetryOutput () {
        telemetry.addData("LED Pos VAR: ", ledPosition);
//        telemetry.addData("LED getPosition: ", ledStrip.getPosition());

        telemetry.addData("DRIVE MODE ", driveMode);
        telemetry.addData("DRIVE FORWARD: ", driveForward);
//        telemetry.addData("Left Y: ", leftY);
//        telemetry.addData("Right Y: ", rightY);
        telemetry.addLine("D_PAD RIGHT FOR ARCADE MODE");
        telemetry.addLine("D_PAD LEFT FOR TANK DRIVE");
        telemetry.addLine("D_UP FOR FORWARD MODE");
        telemetry.addLine("D_DOWN FOR REVERSE MODE");
    }
}


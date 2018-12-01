package org.firstinspires.ftc.teamcode.robot.outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.CatapultArm;


import static java.lang.Thread.sleep;
//adb connect 1.2.3.4:5555

@TeleOp(name = "Outreach Robot")
@Disabled
public class OutreachRobot extends OpMode {

    OutreachMotors myOutreachMotors;
    //outreachCatapultMotorsRunToPosition myOutreachCatapultMotorsRunToPosition;
    //outreachCatapultMotorsRunUsingEncoders myOutreachCatapultMotorsRunUsingEncoders;
    //outreachCatapultMotorsRunWithoutEncoders myOutreachCatapultMotorsRunWithoutEncoders;
    //outreachCatapultMotorsRunUsingEncoders myOutreachCatapultMotorsRunUsingEncoders;
    //outreachCatapultMotorsRunToPosition2motors myOutreachCatapultMotorsRunToPosition2motors;

    CatapultArm myCatapultArm;

    //outreachCatapultMotorsRunWithoutEncoders2motors myOutreachCatapultMotorsRunWithoutEncoders2motors;

    outreachTouchSensorCatapult myOutreachTouchSensorCatapult;
    //value for left joystick
    double leftY;
    //value for right joystick
    double rightY;

    double triggerLeft;
    double triggerRight;
    DcMotor catapult;

    double minLedPosition = 0.2525;
    double maxLedPosition = 0.7475;
    double ledPosition = minLedPosition;
    double ledIncrementValue = 0.0001;

    Servo ledStrip;

    boolean bumperAllow = true;

    @Override
    public void init() {
        myOutreachMotors = new OutreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
//        myCatapultArm = new CatapultArm(hardwareMap.dcMotor.get("left_catapult_motor"), hardwareMap.dcMotor.get("right_catapult_motor"));
//        myOutreachTouchSensorCatapult = new outreachTouchSensorCatapult (hardwareMap.get(DigitalChannel.class, "catapult_touch_sensor"));

        ledStrip = hardwareMap.servo.get("led_strip");
        ledStrip.setPosition(ledPosition);



    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            ledPosition = ledPosition + ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }

        if (gamepad1.left_bumper) {
            ledPosition = ledPosition - ledIncrementValue;
            ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
        }

//        if (gamepad1.left_bumper) {
//            ledPosition = minLedPosition;
//        }
//        if (gamepad1.right_bumper) {
//            ledPosition = maxLedPosition;
//        }

//        if (bumperAllow == true && (gamepad1.left_bumper == true || gamepad1.right_bumper == true)) {
//            if (gamepad1.left_bumper) {
//                ledPosition = ledPosition - .1;
//                ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
//                ledStrip.setPosition(ledPosition);
//            }
//            if (gamepad1.right_bumper) {
//                ledPosition = ledPosition + .1;
//                ledPosition = Range.clip(ledPosition,minLedPosition,maxLedPosition);
//                ledStrip.setPosition(ledPosition);
//            }
//            bumperAllow = false;
//        }
//        else if (!bumperAllow && !gamepad1.left_bumper && !gamepad1.right_bumper) {
//            bumperAllow = true;
//        }

        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        myOutreachMotors.drive(leftY, rightY);
        //trigger left lowers arm
        //trigger right raises arm
//        triggerLeft = gamepad1.left_trigger;
//        triggerRight = gamepad1.right_trigger;
//        myCatapultArm.catapultMotorManualOperation(triggerLeft, triggerRight, myOutreachTouchSensorCatapult);



/*
        if (gamepad1.right_trigger == 1) {
            myCatapultArm.catapultReset(myOutreachTouchSensorCatapult);
            myCatapultArm.motorPowerZero();
            ledPosition = .43;
            ledStrip.setPosition(ledPosition);  //needed because will never get to the IF statement at bottom of loop()
            myCatapultArm.catapultLaunch(myOutreachTouchSensorCatapult);
            myCatapultArm.motorPowerZero();
            myCatapultArm.catapultReset(myOutreachTouchSensorCatapult);
            ledPosition = .4;
            myCatapultArm.motorPowerZero();
        }

        if (gamepad1.left_trigger == 1) {
            myCatapultArm.catapultReset(myOutreachTouchSensorCatapult);
            myCatapultArm.motorPowerZero();
        }

        if (gamepad1.a) {
            myCatapultArm.catapultMotorManualOperation(.3, 0, myOutreachTouchSensorCatapult);
            myCatapultArm.motorPowerZero();
        }

        if (gamepad1.y) {
            myCatapultArm.catapultMotorManualOperation(0, .6, myOutreachTouchSensorCatapult);
            myCatapultArm.motorPowerZero();
        }
*/
        if (ledPosition != ledStrip.getPosition()) {
            ledStrip.setPosition(ledPosition);
        }

        //catapult.setPower(-triggerLeft);  jkjkjkj
        //catapult.setPower(triggerRight);
//        telemetry.addData("Touch Sensor: ", myOutreachTouchSensorCatapult.checkCatapultTouchSensor());
        telemetry.addData("LED Pos VAR: ", ledPosition);
        telemetry.addData("LED getPosition: ", ledStrip.getPosition());
        telemetry.addData("Left Y: ", leftY);
        telemetry.addData("Right Y: ", rightY);
//        telemetry.addData("Left trigger: ", gamepad1.left_trigger);
//        telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
//        telemetry.update();
    }
}

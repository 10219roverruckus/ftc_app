package org.firstinspires.ftc.teamcode.outreach.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import static java.lang.Thread.sleep;


@TeleOp(name = "Demo Robot")
public class outreachRobot extends OpMode {

    outreachMotors myOutreachMotors;
    //outreachCatapultMotorsRunToPosition myOutreachCatapultMotorsRunToPosition;
    //outreachCatapultMotorsRunUsingEncoders myOutreachCatapultMotorsRunUsingEncoders;
    //outreachCatapultMotorsRunWithoutEncoders myOutreachCatapultMotorsRunWithoutEncoders;
    outreachCatapultMotorsRunWithoutEncoders2motors myOutreachCatapultMotorsRunWithoutEncoders2motors;

    outreachTouchSensorCatapult myOutreachTouchSensorCatapult;
    //value for left joystick
    double leftY;
    //value for right joystick
    double rightY;

    double triggerLeft;
    double triggerRight;
    DcMotor catapult;



    @Override
    public void init() {
        myOutreachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        //myOutreachCatapultMotorsRunToPosition = new outreachCatapultMotorsRunToPosition(hardwareMap.dcMotor.get("catapult_motor"));
        //myOutreachCatapultMotorsRunUsingEncoders = new outreachCatapultMotorsRunUsingEncoders(hardwareMap.dcMotor.get("catapult_motor"));
        //myOutreachCatapultMotorsRunWithoutEncoders = new outreachCatapultMotorsRunWithoutEncoders(hardwareMap.dcMotor.get("catapult_motor"));
        myOutreachCatapultMotorsRunWithoutEncoders2motors = new outreachCatapultMotorsRunWithoutEncoders2motors (hardwareMap.dcMotor.get("catapult_motor1"), hardwareMap.dcMotor.get("catapult_motor2"));
        myOutreachTouchSensorCatapult = new outreachTouchSensorCatapult (hardwareMap.get(DigitalChannel.class, "catapult_touch_sensor"));
        //catapult = hardwareMap.dcMotor.get("catapult_motor");
        //catapult.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        myOutreachMotors.drive(leftY, rightY);
        //trigger left lowers arm
        //trigger right raises arm
        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;
//        myOutreachCatapultMotorsRunToPosition.catapultMotorManualOperation (triggerLeft, triggerRight, myOutreachTouchSensorCatapult);
        myOutreachCatapultMotorsRunWithoutEncoders2motors.catapultMotorManualOperation (triggerLeft, triggerRight, myOutreachTouchSensorCatapult);


        if (gamepad1.left_bumper) {
            myOutreachCatapultMotorsRunWithoutEncoders2motors.catapultReset(myOutreachTouchSensorCatapult);
        }

        if (gamepad1.right_bumper) {
    //        myOutreachCatapultMotorsRunToPosition.catapultLaunch(myOutreachTouchSensorCatapult);
        }

        if (gamepad1.y) {
            myOutreachCatapultMotorsRunWithoutEncoders2motors.catapultLaunch(myOutreachTouchSensorCatapult);
            try {
                sleep (100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //myOutreachCatapultMotorsRunWithoutEncoders2motors.catapultReset(myOutreachTouchSensorCatapult);
            myOutreachCatapultMotorsRunWithoutEncoders2motors.catapultStopMotors();
        }

        if (gamepad1.a) {
      //      myOutreachCatapultMotorsRunUsingEncoders.catapultLaunch(myOutreachTouchSensorCatapult);
        }
        //catapult.setPower(-triggerLeft);
        //catapult.setPower(triggerRight);
        telemetry.addData("Touch Sensor: ", myOutreachTouchSensorCatapult.checkCatapultTouchSensor());
        telemetry.addData("Left Y: ", leftY);
        telemetry.addData("Right Y: ", rightY);
        telemetry.addData("Left trigger: ", triggerLeft);
        telemetry.addData("Right Trigger: ", triggerRight);
        telemetry.update();
    }
}

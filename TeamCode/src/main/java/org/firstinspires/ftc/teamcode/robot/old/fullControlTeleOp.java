package org.firstinspires.ftc.teamcode.robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.old.classes.sub.boardArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.colorSensorArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphArms;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphLift;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.relicArm;

/**
 * Created by blake_shafer on 8/23/17.
 */

@Disabled
@TeleOp(name = "Full Control (OLD RELIC RECOVERY)")

public class fullControlTeleOp extends OpMode {

    // left stick y axis controls forward/backward rotation of left motors
    // right stick y axis controls forward/backward rotation of right motors (tank drive)
    // left/right triggers control strafing left/right

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double leftStickVal;
    double rightStickVal;

    double leftTriggerVal;
    double rightTriggerVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    boolean relicMode;

    double liftPower;
    int position;
    int minLiftPosition = 0;
    int maxLiftPosition = 9200;

    double relicExtensionPower;
    double relicGrabberPosition;

    boolean initServos = false;

    glyphArms myGlyphArms;
    colorSensorArm myColorSensorArm;
    glyphLift myGlyphLift;
    boardArm myBoardArm;
    relicArm myRelicArm;

    @Override
    public void init() {

        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_left_glyph_arm"), hardwareMap.servo.get("top_right_glyph_arm"), hardwareMap.servo.get("bottom_right_glyph_arm"));
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("rev_color_sensor_arm"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myBoardArm = new boardArm(hardwareMap.dcMotor.get("board_arm"));
        myRelicArm = new relicArm(hardwareMap.dcMotor.get("relic_arm_lift"), hardwareMap.dcMotor.get("relic_arm_extension"), hardwareMap.servo.get("relic_arm_grabber"));

        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicMode = false;
    }
    @Override
    public void loop() {

        if (!initServos) {
            myGlyphArms.openLoweredGlyphArms();
            myColorSensorArm.colorSensorArmUp();
            myColorSensorArm.colorRotateResting();
            myRelicArm.relicGrabberOpen();
            initServos = true;
        }

        if (gamepad2.b) {
            myBoardArm.boardArmDown();
        }

        else if (gamepad2.y) {
            myBoardArm.boardArmUp();
        }

        else {
            myBoardArm.boardArmStop();
        }

        if (gamepad1.dpad_up) {
            relicMode = false;
        }

        else if (gamepad1.dpad_down) {
            relicMode = true;
        }

        leftStickVal = -gamepad1.left_stick_y;
        leftStickVal = Range.clip(leftStickVal, -1, 1);
        rightStickVal = -gamepad1.right_stick_y;
        rightStickVal = Range.clip(rightStickVal, -1, 1);

        leftTriggerVal = gamepad1.left_trigger;
        leftTriggerVal = Range.clip(leftTriggerVal, 0, 1);
        rightTriggerVal = gamepad1.right_trigger;
        rightTriggerVal = Range.clip(rightTriggerVal, 0, 1);

        if (relicMode) {
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

            frontLeftSpeed = leftStickVal - leftTriggerVal + rightTriggerVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = rightStickVal - rightTriggerVal + leftTriggerVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickVal + leftTriggerVal - rightTriggerVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = rightStickVal + rightTriggerVal - leftTriggerVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            frontLeftMotor.setPower(rearRightSpeed);
            frontRightMotor.setPower(rearLeftSpeed);
            rearLeftMotor.setPower(frontRightSpeed);
            rearRightMotor.setPower(frontLeftSpeed);

            myGlyphArms.relicGlyphArms();
        }

        else {
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

            frontLeftSpeed = leftStickVal - leftTriggerVal + rightTriggerVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = rightStickVal - rightTriggerVal + leftTriggerVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickVal + leftTriggerVal - rightTriggerVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = rightStickVal + rightTriggerVal - leftTriggerVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            frontLeftMotor.setPower(frontLeftSpeed);
            frontRightMotor.setPower(frontRightSpeed);
            rearLeftMotor.setPower(rearLeftSpeed);
            rearRightMotor.setPower(rearRightSpeed);
        }

        // Glyph Arms

        if (gamepad2.left_bumper) {
            myGlyphArms.openLoweredGlyphArms();
        }
        if (gamepad2.right_bumper) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.a) {
            myGlyphArms.slightlyOpenGlyphArms();
        }
        if (gamepad2.x) {
            myGlyphArms.openRaisedGlyphArms();
        }

        // Glyph Lift

        liftPower = -gamepad2.left_stick_y;
        liftPower = Range.clip(liftPower, -1, 1);

        myGlyphLift.setPower(liftPower);

        // Relic Arm

        if (gamepad2.dpad_up) {
            myRelicArm.setLiftPower(1);
        }

        else if (gamepad2.dpad_down) {
            myRelicArm.setLiftPower(-1);
        }

        else {
            myRelicArm.setLiftPower(0);
        }

        relicExtensionPower = -gamepad2.right_stick_y;
        relicExtensionPower = Range.clip(relicExtensionPower, -1, 1);

        myRelicArm.setExtensionPower(relicExtensionPower);

        relicGrabberPosition = myRelicArm.getRelicGrabberPosition();

        if (gamepad2.left_trigger >= 0.2) {
            relicGrabberPosition = relicGrabberPosition - 0.005;
        }

        if (gamepad2.right_trigger >= 0.2) {
            relicGrabberPosition = relicGrabberPosition + 0.005;
        }

        myRelicArm.setRelicGrabberPosition(relicGrabberPosition);

        if (relicGrabberPosition <= 0) {
            relicGrabberPosition = 0;
        }

        else if (relicGrabberPosition >= 0.94) {
            relicGrabberPosition = 0.94;
        }

        if (gamepad2.right_stick_button) {
            myRelicArm.relicGrabberAlmostClose();
        }

        // Telemetry

        //telemetry.addData("val", "L stck: " + leftStickVal);
        //telemetry.addData("val", "R stck: " + rightStickVal);
        //telemetry.addData("val", "L trgr: " + leftTriggerVal);
        //telemetry.addData("val", "R trgr: " + rightTriggerVal);

        //telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
        //telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
        //telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
        //telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);

        telemetry.addData("Relic Arm Power: ", relicExtensionPower);
        telemetry.addData("Relic Grabber Position: ", relicGrabberPosition);
        telemetry.update();
    }
}

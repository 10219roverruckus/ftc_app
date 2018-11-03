package org.firstinspires.ftc.teamcode.robot.competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.boardArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.colorSensorArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphArms;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphLift;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.relicArm;

/**
 * Created by blake_shafer on 8/23/17.
 */

//@Disabled
@TeleOp(name = "Full Control - Main Channel Robot")

public class FullControlTeleOp extends OpMode {

    // left stick y axis controls forward/backward rotation of left motors
    // right stick y axis controls forward/backward rotation of right motors (tank drive)
    // left/right triggers control strafing left/right

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
    DcMotor liftArmMotor;
    DcMotor intakePositionMotor;
    DcMotor intakeMotor;


    Servo teamMarkerArm;
    Servo intakeExtenderArm;



    double leftStickVal;
    double rightStickVal;

    double leftTriggerVal;
    double rightTriggerVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double rightJoystick_lift;

    double intakeExtensionPower;
    double intakePositionPower;
    double intakePosition = 0;
    double intakeIncrement = .001;
    double teamMarkerPosition = .76;

    boolean reverseMode;

    boolean initServos;

    double powerThreshold = 0.1;

    double topHeight = 8.4;
    double lowHeight = 3.2;

    //boolean liftSensorOverride = false;


//    private DistanceSensor liftDistanceSensor;

     LiftMotor myLiftMotor;
     IntakeExtenderArm myIntakeExtenderArm;
//    IntakeRotator myIntakeRotator;

    final double SPD_DRIVE_LOW = .20;     //Lowest speed
    final double SPD_DRIVE_MED = .5;      //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 200;


    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");
        liftArmMotor = hardwareMap.dcMotor.get("lift_motor");
     //   liftDistanceSensor = hardwareMap.get(DistanceSensor.class, "lift_distance_sensor");
        intakePositionMotor = hardwareMap.dcMotor.get("intake_position_motor");
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        intakePositionMotor.setDirection(DcMotor.Direction.REVERSE);

        teamMarkerArm = hardwareMap.servo.get("team_marker_arm");
        intakeExtenderArm = hardwareMap.servo.get("intake_extender_arm");

        // need to initilize sensors here

        reverseMode = false;

        initServos = false;
        teamMarkerArm.setPosition(teamMarkerPosition);
        intakeExtenderArm.setPosition(.5);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {    //see if the controller is in reverse mode or not (if joysticks are pressed down or not)
            reverseMode = false; // forward mode
        }

        else if (gamepad1.dpad_down) {
            reverseMode = true;    //reverse mode
        }

        drive();


        //align rotator motor


        rightJoystick_lift = gamepad2.right_stick_y;    //assigns rotator to right stick y

        if (rightJoystick_lift < -.1 || rightJoystick_lift > .1) {
            intakePositionMotor.setPower(rightJoystick_lift);
        }
        else {
            intakePositionMotor.setPower(0);
        }

        //lift motor up and down

        if (gamepad2.dpad_down) {
            liftArmMotor.setPower(-1);
        }
        else if (gamepad2.dpad_up) {
            liftArmMotor.setPower(1);
        }
        else {
            liftArmMotor.setPower(0);
        }

        // extender arm for out and in




        if (gamepad2.left_stick_y > .1) {
//            intakePosition = intakePosition + intakeIncrement;
//            intakeExtenderArm.setPosition(intakePosition);
            intakeExtenderArm.setPosition(1);
        }
        else if (gamepad2.left_stick_y < -.1) {
//            intakePosition = intakePosition - intakeIncrement;
//            intakeExtenderArm.setPosition(intakePosition);
            intakeExtenderArm.setPosition(0);
        }
        else {
            intakeExtenderArm.setPosition(.5);
        }



        //THIS CAN COME OUT
        if (gamepad1.right_bumper) {
            teamMarkerPosition = teamMarkerPosition + intakeIncrement;
            teamMarkerArm.setPosition(teamMarkerPosition);
        }
        else if (gamepad1.left_bumper) {
            teamMarkerPosition = teamMarkerPosition - intakeIncrement;
            teamMarkerArm.setPosition(teamMarkerPosition);
        }



        if (gamepad2.right_trigger > .1) {
            intakeMotor.setPower(1);
        }
        else if (gamepad2.right_trigger < -.1) {
            intakeMotor.setPower(-1);
        }
        else {
        intakeMotor.setPower(0);
    }
    telemetryOutput();
    }


    public void telemetryOutput (){
        telemetry.addData("EXTENDER POSITION SERVO: ", intakeExtenderArm.getPosition());
        telemetry.addData("TEAM MARKER SERVO: ", teamMarkerArm.getPosition());
//        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
//        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
//        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
//        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);
//        telemetry.update();

        // generic DistanceSensor methods.
//        telemetry.addData("deviceName",liftDistanceSensor.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", liftDistanceSensor.getDistance(DistanceUnit.MM)));
//        telemetry.addData("range", String.format("%.01f cm", liftDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range", String.format("%.01f m", liftDistanceSensor.getDistance(DistanceUnit.METER)));
//        telemetry.addData("range", String.format("%.01f in", liftDistanceSensor.getDistance(DistanceUnit.INCH)));

//        telemetry.update();

    }

    public void drive () {
        leftStickVal = -gamepad1.left_stick_y;
        leftStickVal = Range.clip(leftStickVal, -1, 1);
        rightStickVal = -gamepad1.right_stick_y;
        rightStickVal = Range.clip(rightStickVal, -1, 1);


        leftTriggerVal = gamepad1.left_trigger;
        leftTriggerVal = Range.clip(leftTriggerVal, 0, 1);
        rightTriggerVal = gamepad1.right_trigger;
        rightTriggerVal = Range.clip(rightTriggerVal, 0, 1);

        if (reverseMode) {
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

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                frontLeftMotor.setPower(frontLeftSpeed);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold){
                frontRightSpeed = 0;
                frontRightMotor.setPower(frontRightSpeed);
            } else {
                frontRightMotor.setPower(frontRightSpeed);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                rearLeftMotor.setPower(rearLeftSpeed);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold){
                rearRightSpeed = 0;
                rearRightMotor.setPower(rearRightSpeed);
            } else {
                rearRightMotor.setPower(rearRightSpeed);
            }
        }
    }
}

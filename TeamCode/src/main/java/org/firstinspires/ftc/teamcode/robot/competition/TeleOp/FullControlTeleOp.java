package org.firstinspires.ftc.teamcode.robot.competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
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

    boolean reverseMode;

    boolean initServos;

    double powerThreshold = 0.1;

    LiftMotor myLiftMotor;
    IntakeExtenderArm myIntakeExtenderArm;

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


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        teamMarkerArm = hardwareMap.servo.get("team_marker_arm");
        intakeExtenderArm = hardwareMap.servo.get("intake_extender_arm");

        // need to initilize sensors here

        reverseMode = false;

        initServos = false;
    }


    //Code runs only ONCE when driver hits PLAY

    @Override
    public void start () {
        //SET LIFT MOTOR TO RETRACTED STATE
        myLiftMotor.retractLiftMotorFully();
        // SET INTAKE ARM TO RETRACTED STATE
//        myIntakeExtenderArm.retractingIntakeArm(SPD_ARM_MED, 1);
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

        rightJoystick_lift = gamepad2.right_stick_y;    //assigns liftto right stick y

        if (rightJoystick_lift < -.2 || rightJoystick_lift > .2) {
            liftArmMotor.setPower(rightJoystick_lift);
        }
        else {
            liftArmMotor.setPower(0);   //if right joystick is within the two values then the power is 0
        }

        // intake system
        intakeExtensionPower = gamepad2.right_stick_y;      // WIP
        intakePositionPower = gamepad2.left_stick_y;










        //MOTOR FOR LIFT

        // Telemetry

        //telemetry.addData("val", "L stck: " + leftStickVal);
        //telemetry.addData("val", "R stck: " + rightStickVal);
        //telemetry.addData("val", "L trgr: " + leftTriggerVal);
        //telemetry.addData("val", "R trgr: " + rightTriggerVal);

        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);

        telemetry.update();
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

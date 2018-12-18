package org.firstinspires.ftc.teamcode.robot.competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;


/**
 * Created by blake_shafer on 8/23/17.
 */

//@Disabled
@TeleOp(name = "Full Control - Worlds Robot1")

public class FullControlTeleOp extends OpMode {

    // left stick y axis controls forward/backward rotation of left motors
    // right stick y axis controls forward/backward rotation of right motors (tank drive)
    // left/right triggers control strafing left/right

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;
//    DcMotor liftArmMotor;

//    DcMotor intakeRotater;


//    Servo intakeServoL;
//    Servo intakeServoR;
    //DcMotor intakeMotor;

   // DcMotor myintakeExtenderArm;



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

    boolean reverseModeToggle;

    boolean initServos;

    double powerThreshold = 0.1;

    double topHeight = 8.4;
    double lowHeight = 3.2;

    //boolean liftSensorOverride = false;


//    private DistanceSensor liftDistanceSensor;

    LiftMotor myLiftMotor;
    IntakeExtenderArm myIntakeExtenderArm;
    IntakeRotator myIntakeRotator;
    IntakeServo myIntakeServo;
    RevColorDistance myRevColorDistance;

//    LanderMotor myLanderMotor;
    // 3 servos...
    // 2 for rotating lander scorer
    // 1 transfer mineral gate
    LanderServo myLanderServo;
    //lifts x-rails to the lander - MOTOR
    MineralLift myMineralLift;


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

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myLiftMotor = new LiftMotor(hardwareMap.dcMotor.get("lift_motor"));
        myIntakeExtenderArm  = new IntakeExtenderArm (hardwareMap.dcMotor.get("intake_extender_arm"));
        myIntakeRotator = new IntakeRotator(hardwareMap.dcMotor.get("intake_rotater_motor"));
        myIntakeServo = new IntakeServo(hardwareMap.servo.get("intake_spinner_servo_left"), hardwareMap.servo.get("intake_spinner_servo_right"));
        myRevColorDistance = new RevColorDistance(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));
//        myLanderMotor = new LanderMotor(hardwareMap.dcMotor.get("mineral_lift_motor"));
        myLanderServo = new LanderServo (hardwareMap.servo.get("right_mineral_dumper"), hardwareMap.servo.get("left_mineral_dumper"), hardwareMap.servo.get("transfer_gate_servo"));




        // need to initilize sensors here

        reverseModeToggle = false;

        initServos = false;


    }

    @Override
    public void loop() {

        //reverse mode - reverse DRIVE CONTROL motors.
        reverseMode();


        //drive robot
        drive();

        //controls motor to lift and lower robot
        hangingLiftMotor();

        //controls extending arm for intake mechansim.
        extenderArm();


        //control spinnding hungyr hungry hippo
        spinnerIntake();

        // rotate the intake up and down
        rotater();

        // mineral lift raises and lowers the x rail for dropping the minerals in teh lander
        mineralLift();

        //over ride for dumping the minerals into the tray
        IntakeTransfer();

        // dumps the minerals into the lander when the lift is at the top
        mineralDump();


        //telemetryOutput();
    }


    public void telemetryOutput (){
        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);
        telemetry.update();

//         generic DistanceSensor methods.
//        telemetry.addData("deviceName",liftDistanceSensor.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", liftDistanceSensor.getDistance(DistanceUnit.MM)));
//        telemetry.addData("range", String.format("%.01f cm", liftDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range", String.format("%.01f m", liftDistanceSensor.getDistance(DistanceUnit.METER)));
//        telemetry.addData("range", String.format("%.01f in", liftDistanceSensor.getDistance(DistanceUnit.INCH)));

        telemetry.update();

    }

    //controls motor to lift and lower robot
    public void hangingLiftMotor () {
        if (gamepad2.dpad_down) {
            myLiftMotor.retractLift();
        }
        else if (gamepad2.dpad_up) {
            myLiftMotor.extendLift();
        }
        else {
            myLiftMotor.stopMotors();
        }
    }

    // extender arm for out and in
    public void extenderArm () {
        if (gamepad2.left_stick_y > .1) {
            myIntakeExtenderArm.extendIntakeArm(gamepad2.left_stick_y);
        }
        else if (gamepad2.left_stick_y < -.1) {
            myIntakeExtenderArm.retractIntactArm(gamepad2.left_stick_y);
        }
        else {
            myIntakeExtenderArm.stopIntakeArm();
        }
    }

    public void spinnerIntake () {
        if (gamepad2.right_trigger > .1) {
            myIntakeServo.IntakeServoForward();
        }
        else if (gamepad2.left_trigger > .1) {
            myIntakeServo.IntakeServoReverse();
        }
        else {
            myIntakeServo.stopIntakeServo();
        }
    }

    public void IntakeTransfer () {
//        if (gamepad2.a == true || colorsenor.red > 10) {
        if (gamepad2.a == true) {
            myLanderServo.releaseMinerals();
        }
        else {
            myLanderServo.keepMineralsIn();
        }
    }

    public void mineralDump () {
        if (gamepad2.y == true) {
            myLanderServo.landerServoScore();
        }
        else {
            myLanderServo.landerServoCollect();
        }
    }

    public void rotater () {
        if (gamepad2.left_bumper == true) {
            myIntakeRotator.RaiseIntakeRotater();
        }
        else if (gamepad2.left_bumper == true) {
            myIntakeRotator.LowerIntakeRotater();
        }
        else {
            myIntakeRotator.stopIntakeRotatorMotors();
        }
    }

    public void mineralLift () {
        if (gamepad2.left_stick_y > .1) {
            myMineralLift.RaiseMineralLift(gamepad2.left_stick_y);
        }
        else if (gamepad2.left_stick_y < -.1) {
            myMineralLift.LowerMineralLift(gamepad2.left_stick_y);
        }
        else {
            myMineralLift.stopIntakeMotors();
        }
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

        if (reverseModeToggle) {
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

    //Reverse mode?
    public void reverseMode () {
        if (gamepad1.dpad_up) {    //see if the controller is in reverse mode or not (if joysticks are pressed down or not)
            reverseModeToggle = false; // forward mode
        }

        else if (gamepad1.dpad_down) {
            reverseModeToggle = true;    //reverse mode
        }
    }
}

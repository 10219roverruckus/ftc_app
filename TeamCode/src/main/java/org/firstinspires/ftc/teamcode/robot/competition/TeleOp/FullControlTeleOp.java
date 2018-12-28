package org.firstinspires.ftc.teamcode.robot.competition.TeleOp;

import android.graphics.Color;
import android.graphics.ColorFilter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.LEDLights;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;


/**
 * Created by the team on 8/23/17.
 */

//@Disabled
@TeleOp(name = "Full Control - Worlds Robot")

public class FullControlTeleOp extends OpMode {

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

    boolean reverseModeToggle;

    boolean initTeleOpToggle;


    double powerThreshold = 0.1;

    public ElapsedTime TeleOpTime;
//    public double normalGameTime = 1300;
    public double endGameTime = 90000;
    public double hangTime = 108000;



    LiftMotor myLiftMotor;
    IntakeExtenderArm myIntakeExtenderArm;
    IntakeRotator myIntakeRotator;
    IntakeServo myIntakeServo;
    //lifts x-rails to the lander - MOTOR
    MineralLift myMineralLift;
    // 3 servos...
    // 2 for rotating lander scorer
    // 1 transfer mineral gate
    LanderServo myLanderServo;
    RevColorDistance myRevColorDistance;
    LEDLights myLEDStrip;

    boolean mineralLiftAllowed;

    boolean hookLiftAllowed;

    boolean extenderAllowed;

    int LEDRed = 1000;
    int LEDBlue = 1000;
    int LEDGreen = 1000;

    @Override
    public void init() {

        //map  & set up devices.
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
        myMineralLift = new MineralLift(hardwareMap.dcMotor.get("mineral_lift_motor"));
        myLanderServo = new LanderServo (hardwareMap.servo.get("right_mineral_dumper"), hardwareMap.servo.get("left_mineral_dumper"), hardwareMap.servo.get("transfer_gate_servo"));

        myLEDStrip = new LEDLights(hardwareMap.servo.get("led_strip"));
        myRevColorDistance = new RevColorDistance(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_mineral_lift"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_mineral_lift"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_hook"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_hook"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_extender"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_extender"));


        //set initial toggles
        reverseModeToggle = false;
        initTeleOpToggle = true;
    }

    @Override
    public void loop() {

        if (initTeleOpToggle == true) {
            initTeleOp();
        }
        else {
            initTeleOpToggle = false;
        }

        //reverse mode - reverse DRIVE CONTROL motors.
        reverseMode();


        //drive robot
        drive();

        //controls motor to lift and lower robot
        hangingLiftMotor();

        //controls extending arm for intake mechansim.
        extenderArm();

        //control spinning hungry hungry hippo doohikkie
        spinnerIntake();

        // rotate the intake up and down
        rotater();

        // mineral lift raises and lowers the x rail for dropping the minerals in teh lander
        mineralLift();

        //over ride for dumping the minerals into the tray
        IntakeTransfer();

        // dumps the minerals into the lander when the lift is at the top
        mineralDump();

        //output telemetry
        telemetryOutput();

        // LED lights for lift (hook)
        //EncoderColorChangesHookLift();

        // LED lights for mineral lift
        //EncoderColorChangesMineralLift();

        // LED lights for extender
        //EnocderColorChangesExtenderLift();

        //LED lights for the time in the game
        TimingInTeleOpWithLED();

    }


    public void telemetryOutput (){
//        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
//        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
//        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
//        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);
        telemetry.addData("Left joystick Y (gp2): ", gamepad2.left_stick_y);
        telemetry.addData("Right joystick Y (gp2): ", gamepad2.right_stick_y);
        telemetry.update();

//         generic DistanceSensor methods.
//        telemetry.addData("deviceName",liftDistanceSensor.getDeviceName() );
//        telemetry.addData("range", String.format("%.01f mm", liftDistanceSensor.getDistance(DistanceUnit.MM)));
//        telemetry.addData("range", String.format("%.01f cm", liftDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range", String.format("%.01f m", liftDistanceSensor.getDistance(DistanceUnit.METER)));
//        telemetry.addData("range", String.format("%.01f in", liftDistanceSensor.getDistance(DistanceUnit.INCH)));

        telemetry.update();

    }

    // reset things
    public void initTeleOp () {
        TeleOpTime.reset();
    }

    //controls motor to lift and lower robot
    public void hangingLiftMotor () {
        if (gamepad2.dpad_down && myRevColorDistance.checkSensorHookLift()) {
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
        else if (gamepad2.left_stick_y < -.1 && myRevColorDistance.checkSensorExtender() == true) {
            myIntakeExtenderArm.retractIntactArm(gamepad2.left_stick_y);
        }
        else {
            myIntakeExtenderArm.stopIntakeArm();
        }
    }

    public void spinnerIntake () {
        if (gamepad2.right_trigger > powerThreshold) {
            myIntakeServo.IntakeServoForward();
        }
        else if (gamepad2.left_trigger > powerThreshold) {
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
        else if (gamepad2.right_bumper == true) {
            myIntakeRotator.LowerIntakeRotater();
        }
        else {
            myIntakeRotator.stopIntakeRotatorMotors();
        }
    }

    public void mineralLift () {

        if (gamepad2.left_stick_y > powerThreshold) {
            myMineralLift.RaiseMineralLift(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < -powerThreshold && myRevColorDistance.checkSensorMineralLift() == true) {
            myMineralLift.LowerMineralLift(gamepad2.left_stick_y);
        } else {
            myMineralLift.stopMotors();
        }
    }


    //LED light changes for positions of mechanisms

        public void EncoderColorChangesMineralLift () {
            if (myMineralLift.mineralLift.getCurrentPosition() < 100) {         //Mineral lift lowered = color red
                myLEDStrip.LEDred();

            }
            else if (myMineralLift.mineralLift.getCurrentPosition() < 900) {    //Mineral lift in the middle = green
                myLEDStrip.LEDgreen();
            }
            else {                                                              // MIneral lift at the top = blue
                myLEDStrip.LEDblue();
            }
        }

        public void EncoderColorChangesHookLift () {
            if (myLiftMotor.liftMotor.getCurrentPosition() < 100) {             // Hook lift lowered = red
                myLEDStrip.LEDred();
            }
            else if (myLiftMotor.liftMotor.getCurrentPosition() < 900) {
                myLEDStrip.LEDblue();
            }
            else {
                myLEDStrip.LEDgreen();
            }
        }

        public void EnocderColorChangesExtenderLift () {
            if (myIntakeExtenderArm.intakeExtenderArm.getCurrentPosition() < 100) {
                myLEDStrip.LEDred();
            }
            else if (myIntakeExtenderArm.intakeExtenderArm.getCurrentPosition() < 900) {
                myLEDStrip.LEDblue();
            }
            else {
                myLEDStrip.LEDgreen();
            }
        }


        // LED lights for counting down

    public void TimingInTeleOpWithLED () {
        if (TeleOpTime.time() < endGameTime) {
            myLEDStrip.LEDPurple();
        }
        else if (TeleOpTime.time() < hangTime) {
            myLEDStrip.LEDred();
        }
        else {
            myLEDStrip.LEDgreen();
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
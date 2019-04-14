package org.firstinspires.ftc.teamcode.robot.competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.LEDLights;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;



//@Disabled
@TeleOp(name = "Full Control - Worlds Robot - FPS")

public class FullControlTeleOpFirstPersonDriver extends OpMode {

    // left stick y axis controls forward/backward rotation of left motors
    // right stick y axis controls forward/backward rotation of right motors (tank drive)
    // left/right triggers control strafing left/right

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    boolean reverseModeToggle;
    double speedMultiplier = .6;

    boolean initTeleOpToggle;


    double powerThreshold = 0;

    public ElapsedTime TeleOpTime;
    //    public double normalGameTime = 1300;
    public double endGameTime = 90000;
    public double hangTime = 108000;


    LiftMotor myLiftMotor;
    IntakeExtenderArm myIntakeExtenderArm;
    IntakeRotaterServos myIntakeRotator;
    IntakeSpinnerMotor myIntakeSpinnerMotor;
    //lifts x-rails to the lander - MOTOR
    MineralLift myMineralLift;
    // 3 servos...
    // 2 for rotating lander scorer
    // 1 transfer mineral gate
    LanderServo myLanderServo;
    RevColorDistance myRevColorDistance;
    LEDLights myLEDStrip;

    boolean mineralLiftAllowed;

    boolean LeftBumber = false;

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
        myIntakeExtenderArm = new IntakeExtenderArm(hardwareMap.dcMotor.get("intake_extender_arm"));
        myIntakeRotator = new IntakeRotaterServos (hardwareMap.servo.get("rotator_top"), hardwareMap.servo.get("rotator_bottom"));
        myIntakeSpinnerMotor = new IntakeSpinnerMotor(hardwareMap.dcMotor.get("intake_spinner_motor"));
        myMineralLift = new MineralLift(hardwareMap.dcMotor.get("mineral_lift_motor"));
        myLanderServo = new LanderServo(hardwareMap.servo.get("mineral_dumper"));

        myLEDStrip = new LEDLights(hardwareMap.servo.get("led_strip"));
        myRevColorDistance = new RevColorDistance(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));

        TeleOpTime = new ElapsedTime();

        //set initial toggles
        reverseModeToggle = true;
        initTeleOpToggle = true;
    }

    @Override
    public void loop() {

        if (initTeleOpToggle) {
            initTeleOp();
        }

        //reverse mode - reverse DRIVE CONTROL motors.
        reverseMode();

        //Speed Toggle for driver 1 control (Full Speed or Reduced Speed)
        speedToggle();

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

        // New method that incorporates more driver control and color sensor
        mineralLift();

        // old method using color sensor for mineral lift
        //mineralLift_color();

        //output telemetry
        telemetryOutput();

        // LED lights for lift (hook)
        //EncoderColorChangesHookLift();

        // LED lights for mineral lift
        //EncoderColorChangesMineralLift();

        // LED lights for extender
        //EncoderColorChangesExtenderLift();

        //LED lights for the time in the game

        //NO LONGER NEEDED WITH NO SERVO TRANSFER
        //IntakeTransfer();

        //retractAndExtendExtension();

        TimingInTeleOpWithLED();

    }


    public void telemetryOutput() {
        telemetry.addData("SPEED MULTIPLYER", speedMultiplier);
        telemetry.addData("REVERSE MODE", reverseModeToggle);
        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);
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

    // resets & initializes the servo positions for the Intake Rotator and Lander Scorer

    public void initTeleOp() {
        TeleOpTime.reset();
        myIntakeRotator.raisedRotater();        //initializes intake to top position
        myLanderServo.landerServoCollect();      // initialized dump to upright collect position
        initTeleOpToggle = false;               // false so initializes only once
    }

    //controls motor to lift and lower robot
    public void hangingLiftMotor() {
        if (gamepad2.dpad_down) {                   // && myRevColorDistance.checkSensorHookLift()
            myLiftMotor.retractLift();
        } else if (gamepad2.dpad_up) {
            myLiftMotor.extendLift();
        } else {
            myLiftMotor.stopMotors();
        }
    }

    // extender arm for out and in
    public void extenderArm() {
        if (gamepad2.left_stick_y > .1) {
            myIntakeExtenderArm.retractIntactArm(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < -.1) {            //&& myRevColorDistance.checkSensorExtender() == true
            myIntakeExtenderArm.extendIntakeArm(gamepad2.left_stick_y);
        } else {
            myIntakeExtenderArm.stopIntakeArm();
        }
    }

    public void spinnerIntake() {
        if (gamepad2.right_trigger > powerThreshold) {
            myIntakeSpinnerMotor.intakeSpinner(1);
        } else if (gamepad2.left_trigger > powerThreshold  || gamepad2.a) {
            myIntakeSpinnerMotor.intakeSpinner(-1);
        } else {
            myIntakeSpinnerMotor.stopMotors();
        }
    }


    public void rotater() {
        if (gamepad2.right_stick_y < -.1) {
            myIntakeRotator.loweredRotater();
        } else if (gamepad2.right_stick_y > .1) {
            myIntakeRotator.raisedRotater();
        }
    }

    // Added the below method combining Mineral Dump y button, color sensor, and mineral lift extention/retraction

    public void mineralLift() {
        if (gamepad2.y) {
            myLanderServo.landerServoScore();
            LeftBumber = false;

        } else if (gamepad2.right_bumper) {
            myLanderServo.landerServoTravel();
            myMineralLift.RaiseMineralLift();
            LeftBumber = false;

        } else if (myRevColorDistance.checkSensorMineralLift() == false && (gamepad2.left_bumper || LeftBumber)) {                  // was return true for check color Mineral Lift
            LeftBumber = true;
            myLanderServo.landerServoCollect();
            myMineralLift.LowerMineralLift();
            telemetry.addLine("LOWER LIFT!!");

        } else {
            telemetry.addLine("STOP LIFT");
            LeftBumber = false;
            myMineralLift.stopMotors();
        }
    }

    // ************* Below Method not used.  Archived *************************

    public void mineralLift_color() {
        if (gamepad2.right_bumper) {
            myMineralLift.RaiseMineralLift();
            LeftBumber = false;
        } else if (myRevColorDistance.checkSensorMineralLift() == false && (gamepad2.left_bumper || LeftBumber)) {                  // was return true for check color Mineral Lift
            LeftBumber = true;
            myMineralLift.LowerMineralLift();
            telemetry.addLine("LOWER LIFT!!");

        } else {
            telemetry.addLine("STOP LIFT");
            LeftBumber = false;
            myMineralLift.stopMotors();
        }
    }

    // ************* Below Method not used.  Archived *************************

    public void mineralLift_manual() {
        if (gamepad2.right_bumper) {
            myMineralLift.RaiseMineralLift();
            LeftBumber = false;
        } else if (gamepad2.left_bumper) {
            LeftBumber = true;
            myMineralLift.LowerMineralLift();

        } else {
            myMineralLift.stopMotors();
        }
    }

    // ************* Below Method not used.  Archived *************************

    public void retractAndExtendExtension() {
        if (gamepad2.b == true) {

            myIntakeRotator.raisedRotater();

            if (myRevColorDistance.checkSensorExtender() == false) {
                myIntakeExtenderArm.retractIntactArm(1);
            }


        }
    }



    //no longer needed with no transfer servo

   /*
    public void IntakeTransfer() {
//        if (gamepad2.a == true || colorsenor.red > 10) {
        if (gamepad2.a == true) {
            myLanderServo.releaseMinerals();
        } else {
            myLanderServo.keepMineralsIn();
        }
    }
    */


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

        if (reverseModeToggle) {

            leftStickYVal = -gamepad1.left_stick_y;
            leftStickYVal = Range.clip(leftStickYVal, -1, 1);
            leftStickXVal = gamepad1.left_stick_x;
            leftStickXVal = Range.clip(leftStickXVal, -1, 1);
            rightStickXVal = -gamepad1.right_stick_x;
            rightStickXVal = Range.clip(rightStickXVal, -1, 1);

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

            frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                frontLeftMotor.setPower(frontLeftSpeed * speedMultiplier);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold){
                frontRightSpeed = 0;
                frontRightMotor.setPower(frontRightSpeed);
            } else {
                frontRightMotor.setPower(frontRightSpeed * speedMultiplier);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                rearLeftMotor.setPower(rearLeftSpeed * speedMultiplier);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold){
                rearRightSpeed = 0;
                rearRightMotor.setPower(rearRightSpeed);
            } else {
                rearRightMotor.setPower(rearRightSpeed * speedMultiplier);
            }
        }

        else {

            leftStickYVal = -gamepad1.left_stick_y;
            leftStickYVal = Range.clip(leftStickYVal, -1, 1);
            leftStickXVal = gamepad1.left_stick_x;
            leftStickXVal = Range.clip(leftStickXVal, -1, 1);
            rightStickXVal = gamepad1.right_stick_x;
            rightStickXVal = Range.clip(rightStickXVal, -1, 1);
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

            frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                frontLeftMotor.setPower(frontLeftSpeed * speedMultiplier);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold){
                frontRightSpeed = 0;
                frontRightMotor.setPower(frontRightSpeed);
            } else {
                frontRightMotor.setPower(frontRightSpeed * speedMultiplier);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                rearLeftMotor.setPower(rearLeftSpeed * speedMultiplier);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold){
                rearRightSpeed = 0;
                rearRightMotor.setPower(rearRightSpeed);
            } else {
                rearRightMotor.setPower(rearRightSpeed * speedMultiplier);
            }
        }
    }

    //Reverse mode for driver control

    public void reverseMode () {
        if (gamepad1.dpad_up) {    //see if the controller is in reverse mode or not (if joysticks are pressed down or not)
            reverseModeToggle = false; // forward mode
        }

        else if (gamepad1.dpad_down) {
            reverseModeToggle = true;    //reverse mode
        }
    }

    // Speed Toggle for Driver 1.  Allows Driver to shift between full speed and reduced speed using bumpers
    public void speedToggle () {
        if (gamepad1.left_bumper) {
            speedMultiplier = 1.0;
        }

        else if (gamepad1.right_bumper) {
            speedMultiplier = 0.60;
        }
    }

}
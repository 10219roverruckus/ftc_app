package org.firstinspires.ftc.teamcode.robot.testing.EncoderExamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "ENCODERS - RUN TO POSITION", group = "TESTING")
@Disabled

public class runToPosition extends LinearOpMode {
    DcMotor testMotor;
    double testPower = 1;
    int positionThreshold = 5000;

    @Override
    public void runOpMode() throws InterruptedException {

        testMotor = hardwareMap.dcMotor.get("front_left_motor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while (opModeIsActive()) {
            testMotor.setPower(0);
            testMotor.setTargetPosition(positionThreshold);
            while (testMotor.isBusy() && opModeIsActive()) {
                telemetry.addData("ENCODER: ", testMotor.getCurrentPosition());
                testMotor.setPower(testPower);
                telemetry.update();
            }
            testMotor.setPower(0);
            telemetry.addData("ENCODER: ", testMotor.getCurrentPosition());
            telemetry.addLine("OUT OF WHILE");
            telemetry.update();
            sleep(4000);
            idle();
            requestOpModeStop();
        }
        idle();
    }
}

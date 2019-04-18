package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

@Autonomous(name = "TEST DEBUG AUTO STOP", group = "TESTING")
@Disabled
public class TestingAutonomousSTOP extends LinearOpMode {

//    Gyro myGyro;
    //outreachMotors myOutReachMotors;
//    DriveMotors myDriveMotors;
    DcMotor testMotor;
    double testPower = .5;
    int positionThreshold = -5000;

    @Override
    public void runOpMode() throws InterruptedException {
//        myOutReachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
//        myOutReachMotors.setLinearOp(this);
//        myDriveMotors = new DriveMotors(hardwareMap.dcMotor.get("font_left_motor"));
//        myDriveMotors.setLinearOp(this);
        testMotor = hardwareMap.dcMotor.get("front_left_motor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
//        myGyro.setLinearOp(this);
        waitForStart();
//            while (opModeIsActive() || !isStopRequested()) {
        while (opModeIsActive()) {
            testMotor.setPower(0);
            while (testMotor.getCurrentPosition() >= positionThreshold && opModeIsActive()) {
                telemetry.addData("ENCODER: ", testMotor.getCurrentPosition());
                testMotor.setPower(testPower);
                telemetry.update();
            }
            testMotor.setPower(0);
            telemetry.addLine("OUT OF WHILE");
            telemetry.update();
            sleep(4000);
            idle();
            requestOpModeStop();
        }
        idle();
    }
}

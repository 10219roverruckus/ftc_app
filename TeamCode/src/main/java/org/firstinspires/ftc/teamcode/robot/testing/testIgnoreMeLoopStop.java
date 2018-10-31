package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.testing.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

@Autonomous(name = "TEST DEBUG")
@Disabled
public class testIgnoreMeLoopStop extends LinearOpMode {
    Gyro myGyro;
    //outreachMotors myOutReachMotors;
    DriveMotors myDriveMotors;

    @Override
    public void runOpMode() throws InterruptedException {
//        myOutReachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
//        myOutReachMotors.setLinearOp(this);
        myDriveMotors = new DriveMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        myDriveMotors.setLinearOp(this);

        myGyro = new Gyro(hardwareMap.get(BNO055IMU.class, "imu"));
        myGyro.setLinearOp(this);
        waitForStart();
        while (opModeIsActive() || !isStopRequested()) {
            for (int x = 1; x <= 100000; x++) {
                telemetry.addData("COUNTER: ", x);
                telemetry.update();
            }
            idle();
            //requestOpModeStop();
        }
        idle();
    }
}

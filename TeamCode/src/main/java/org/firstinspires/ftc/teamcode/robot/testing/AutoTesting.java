package org.firstinspires.ftc.teamcode.robot.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;

public class AutoTesting extends LinearOpMode  {

    MecanumDrive myMechDrive;

    //gyro
    Orientation angles;
    Acceleration gravity;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        final long sleepTime = 200;
        final double SPD_DRIVE_MED = .5;


        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));

        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        myMechDrive.driveForward(SPD_DRIVE_MED, .65); // move away from the lander toward crater
        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1);
        myMechDrive.driveForward(SPD_DRIVE_MED, 2.6);
        myMechDrive.rotateLeft(SPD_DRIVE_MED, .7);

        //testing gyro to adjust to face the depot after it reaches the wall
        sleep(100);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle > 5) {
            while (angles.firstAngle > 5) {
                myMechDrive.alignLeftFront(5);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
        } else if (angles.firstAngle < -5) {
            while (angles.firstAngle < -5) {
                myMechDrive.alignRightFront(-5);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
    }
}
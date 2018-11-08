package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.outreach.mechanisms.motors.DriveMotors;

//import org.firstinspires.ftc.teamcode.robot.outreach.outreachMotors;


public class GyroCompetition {
    public Orientation angles;
    public Acceleration gravity;
    public BNO055IMU imu;

    public LinearOpMode linearOp = null;


    public final double TOLERANCE = 1;   //variation from target angle allowed.

    //created in constructior
    //    public BNO055IMU imu;

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public GyroCompetition(BNO055IMU I) {

        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = I;
        imu.initialize(parametersimu);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    // WIP
    // NEED TO ADD CODE FOR WHEN GOING FROM +180 TO -180
    public void gyroOrientMecanum (double angle, MecanumDrive myMechDrive) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        linearOp.telemetry.addLine("READY TO ORIENT WITH GYRO!");
        linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(100); //intentionally long sleep for feedback
        if (angles.firstAngle >= angle + TOLERANCE) {
            while (angles.firstAngle >=  angle + TOLERANCE) {
//                linearOp.telemetry.addLine("GREATER THAN WHILE");
//                linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
//                linearOp.telemetry.update();
                myMechDrive.setMotorPowerRotateRight(.2);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        else if (angles.firstAngle <= angle - TOLERANCE) {
            while (angles.firstAngle <= angle - TOLERANCE) {
//                linearOp.telemetry.addLine("LESS THAN WHILE");
//                linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
//                linearOp.telemetry.update();
                myMechDrive.setMotorPowerRotateLeft(.2);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        linearOp.telemetry.addLine("DONE POSITIOING WITH GYRO");
//        linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
//        linearOp.telemetry.update();
        //linearOp.sleep(1000); //intentionally long sleep for feedback
    }


    public void gyroOrientOutreach (double angle, DriveMotors myDriveMotors) {
//        int x = 1;
        linearOp.telemetry.addLine("gyroORIENTOUTREACH");
        linearOp.telemetry.update();
        //linearOp.sleep(1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        linearOp.telemetry.addLine("READY TO ORIENT WITH GYRO!");
        linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.idle();
        //linearOp.sleep(1000); //intentionally long sleep for feedback
        if (angles.firstAngle >= angle + TOLERANCE) {
            while (angles.firstAngle >=  angle + TOLERANCE) {
//                linearOp.telemetry.addLine("GREATER THAN WHILE");
//                linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
//                linearOp.telemetry.update();
                myDriveMotors.drive(.25,-.25);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.idle();
            }
            myDriveMotors.stopMotors();
            //linearOp.sleep(1000);
        }
        else if (angles.firstAngle <= angle - TOLERANCE) {
            while (angles.firstAngle <= angle - TOLERANCE) {
//                linearOp.telemetry.addLine("LESS THAN WHILE");
//                linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
//                linearOp.telemetry.update();
                myDriveMotors.drive(-.25,.25);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.idle();
            }
            myDriveMotors.stopMotors();
           // linearOp.sleep(1000);
        }
        // NEED TO STOP MOTORS!
        myDriveMotors.stopMotors();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        linearOp.telemetry.addLine("DONE POSITIOING WITH GYRO");
        linearOp.telemetry.addData("Current Position: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(1000); //intentionally long sleep for feedback
    }

}

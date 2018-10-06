package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.competition.constructor.motors.mechDriveAuto;
import org.firstinspires.ftc.teamcode.robot.competition.constructor.motors.liftArm;

public class redCrater extends LinearOpMode {
    private int movement = 0;
    mechDriveAuto myMechDriveAuto;
    liftArm myLiftArm;

    //info for gyro
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //float heading;



    @Override
    public void runOpMode() throws InterruptedException {
        myMechDriveAuto = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myLiftArm = new liftArm(hardwareMap.dcMotor.get("lift_motor"));

        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersimu);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        waitForStart();

        while (opModeIsActive()){
            switch (movement) {
                case 0: //opmode initial actions
                    movement++;
                    break;
                case 1: //land robot
                    myLiftArm.extend();
                    sleep(100);
                    myMechDriveAuto.runToPosition(1, 4,.5);
                    myLiftArm.retract();

                    if (angles.firstAngle > .5){
                        while angles.firstAngle > .5{
                            myMechDriveAuto.powerDrive(5, .15);
                        }
                    }
                    else if (angles.firstAngle < -.5) {
                        while (angles.firstAngle < -.5){
                            myMechDriveAuto.powerDrive(-5, .15);
                        }
                    }
                    else {

                    }

                    movement++;
                    break;
                case 2: //rejust the robot
                    movement++;
                    break;
                case 3://knock of mineral
                    movement++;
                    break;
                case 4: //
                    movement++;
                    break;

            }
        }
    }

}

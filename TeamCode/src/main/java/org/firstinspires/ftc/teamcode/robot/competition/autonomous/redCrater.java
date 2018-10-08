package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
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
    private int Goldposition;

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
        GoldAlignDetector detector;

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersimu);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        waitForStart();

        while (opModeIsActive()){
            switch (movement) {
                case 0: //opmode initial actions
                    if (detector.getXPosition() < 280) {
                        Goldposition = 1;
                    }
                    else if (detector.getXPosition() > 500) {
                        Goldposition = 3;
                    }
                    else {
                        Goldposition = 2;
                    }
                    movement++;
                    break;
                case 1: //land robot
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    myLiftArm.extend();
                    sleep(100);
                    myMechDriveAuto.runToPosition(1, 4,.5);
                    myLiftArm.retract();
                    myMechDriveAuto.runToPosition(1,1, .5 );
                    sleep(100 );
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    if (angles.firstAngle > .5){
                        while (angles.firstAngle > .5){
                            myMechDriveAuto.powerDrive(5, .15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                        }
                    }
                    else if (angles.firstAngle < -.5) {
                        while (angles.firstAngle < -.5){
                            myMechDriveAuto.powerDrive(-5, .15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                        }
                    }
                    myMechDriveAuto.stopMotors();
                    movement++;
                    break;
                case 2: // move to correct mineral
                    switch (Goldposition) {
                        case 1: {
                            myMechDriveAuto.runToPosition(3, 3, .5);
                        }
                        case 3: {
                            myMechDriveAuto.runToPosition(3, 4, .5);
                        }
                        case 2: {
                            myMechDriveAuto.runToPosition(3, 1, .5);
                        }
                    }
                    movement++;
                    break;
                case 3:
                    movement++;
                    break;
                case 4: //
                    movement++;
                    break;

            }
            requestOpModeStop();

        }
    }
}

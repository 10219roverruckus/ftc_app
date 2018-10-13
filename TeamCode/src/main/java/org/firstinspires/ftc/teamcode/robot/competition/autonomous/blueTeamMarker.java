package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.liftArm;

@Autonomous(name = "Blue team Marker", group = "Blue")
public class blueTeamMarker extends LinearOpMode {
    private int movement = 0;
    MecanumDrive myMechDrive;
    liftArm myLiftArm;
    private GoldPosition goldPosition = GoldPosition.MIDDLE;

    //info for gyro
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    TeamMarker myTeamMarker;
    IntakeExtenderArm myIntakeArm;





    @Override
    public void runOpMode() throws InterruptedException {
        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myLiftArm = new liftArm(hardwareMap.dcMotor.get("lift_motor"));

        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        GoldAlignDetector detector;

        // created constant variables that are used for speed (different setting)
        final double SPD_DRIVE_LOW = .20;     //Lowest speed
        final double SPD_DRIVE_MED = .5;      //Default is  SPD_MED
        final double SPD_DRIVE_HIGH = .75;
        final double SPD_DRIVE_MAX = 1.0;
        final double SPD_ARM_MED = .5;
        final long sleepTime = 200;


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
        telemetry.addData("waiting for start", movement);   // telementry is used to display where the code the robot is in the code
        telemetry.update();

        myMechDrive.setLinearOp(this);
        waitForStart();

        telemetry.addData("pressed start", movement);
        telemetry.update();

        //not use camera yet - DELETE WHEN CAMERA IN PLACE!
       // goldPosition = goldPosition.LEFT;


        while (opModeIsActive()){
            switch (movement) {
                case 0: //opmode initial actions and detect Gold Mineral
                    if (detector.getXPosition() < 280) {
                        goldPosition = GoldPosition.LEFT;
                    }
                    else if (detector.getXPosition() > 500) {
                        goldPosition = GoldPosition.RIGHT;
                    }
                    else {
                        goldPosition = GoldPosition.MIDDLE;
                    }
                    movement++;   // Increments to next space
                    break;
                case 1: //land robot and adjust robot and get robot away from the lander, so it can collect minerals
                    telemetry.addData("case START: ", movement);
                    telemetry.update();
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    gravity = imu.getGravity();
//                    //myLiftArm.extend();
//                    sleep(100);
                    myMechDrive.strafeRight(SPD_DRIVE_MED,.5);
                    sleep(sleepTime);
//                    //myLiftArm.retract();
                    myMechDrive.driveForward(SPD_DRIVE_MED, 1); // move away from the landertoward crater
                    sleep(sleepTime);
//                    sleep(1000);
                    myMechDrive.strafeLeft(SPD_DRIVE_MED, .5);
                    sleep(sleepTime);

//                    sleep(100 );
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    if (angles.firstAngle > .5){
//                        while (angles.firstAngle > .5){
//                            myMechDrive.rotateLeft(5, .15);
//                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                        }
//                    }
//                    else if (angles.firstAngle < -.5) {
//                        while (angles.firstAngle < -.5){
//                            myMechDrive.rotateRight(-5, .15);
//                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                        }
//                    }
//                    myMechDrive.stopMotors();
                    telemetry.addData("done with case: ", movement);
                    movement = 2;  // increments to case 2
                    sleep(1000);
                    break;
                case 2:           // move to correct mineral / knock it off
                    telemetry.addData("case START ", movement);
                    telemetry.update();
                    sleep(1000);
                    switch (goldPosition) {
                        case LEFT: { //mineral left
                            telemetry.addLine("Left");
                            telemetry.update();
                            myMechDrive.strafeLeft(SPD_DRIVE_MED, 2);
                            sleep(sleepTime);
                            myMechDrive.driveForward(SPD_DRIVE_MED, 1);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, 1);

                            myMechDrive.rotateLeft(SPD_DRIVE_LOW, .2);
                            myIntakeArm.extendingIntakeArm (SPD_ARM_MED, 1);
                            myTeamMarker.teamMarkerArmLowered();
                            myIntakeArm.retractingIntakeArm(SPD_ARM_MED, -1);
                            break;
                        }
                        case RIGHT: { //mineral right
                            telemetry.addLine("Right");
                            telemetry.update();
                            myMechDrive.strafeRight(SPD_DRIVE_MED, 2);
                            sleep(sleepTime);
                            myMechDrive.driveForward(SPD_DRIVE_MED, 1);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, 1);

                            myMechDrive.rotateRight(SPD_DRIVE_LOW, .2);
                            myIntakeArm.extendingIntakeArm (SPD_ARM_MED, 1);
                            myTeamMarker.teamMarkerArmLowered();
                            myIntakeArm.retractingIntakeArm(SPD_ARM_MED, -1);
                            break;
                        }
                        case MIDDLE: { // mineral straight
                            telemetry.addLine("Middle");
                            telemetry.update();
                            myMechDrive.driveForward(SPD_DRIVE_MED, 1);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, 1);

                            myIntakeArm.extendingIntakeArm (SPD_ARM_MED, 1);
                            myTeamMarker.teamMarkerArmLowered();
                            myIntakeArm.retractingIntakeArm(SPD_ARM_MED, -1);
                            break;
                        }
                    }
                    sleep(2000);
                    movement++;
                    break;
                case 3: //Vuphoria  we don't know how to do this part yet
                    myMechDrive.rotateLeft(SPD_DRIVE_MED, .3);

                    movement++;
                    break;

//                case 4: // place team marker / servo arm
//                    myTeamMarker.teamMarkerArmLowered();
//                    sleep(100);
//                    myTeamMarker.teamMarkerArmRaised();
//                    movement++;
//                    break;
//
//                case 5: // park in crater need to look at different pathways that we could take
//
////                    myMechDrive.TeamMarkerToCrater();
////                    movement++;
////                    break;

            }

            idle();
//            requestOpModeStop();

        }

    }
}

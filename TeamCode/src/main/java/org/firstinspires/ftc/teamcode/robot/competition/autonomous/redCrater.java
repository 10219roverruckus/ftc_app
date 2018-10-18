package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.MineralLift;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.liftArm;
@Autonomous(name = "Crater Both", group = "Red - Blue")
public class redCrater extends LinearOpMode {
    private int movement = 0;
    private GoldPosition goldPosition = GoldPosition.MIDDLE;

    //info for gyro
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    TeamMarker myTeamMarker;
    MecanumDrive myMechDrive;
    MineralLift myMineralLift;
    LiftMotor myLiftMotor;
    IntakeMotor myIntakeMotor;
    IntakeExtenderArm myIntakeExtenderArm;
    IntakeRotator myIntakeRotator;
    DistanceSensor liftDistanceSensor;





    @Override
    public void runOpMode() throws InterruptedException {
        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myLiftMotor = new LiftMotor(hardwareMap.dcMotor.get("lift_motor"));
        myTeamMarker = new TeamMarker(hardwareMap.servo.get("team_marker_arm"));
        liftDistanceSensor = hardwareMap.get(DistanceSensor.class, "lift_distance_sensor");

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
        final double SPD_DRIVE_MED = .4;      //Default is  SPD_MED
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

        myTeamMarker.teamMarkerArmRaised();
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

                    goldPosition = GoldPosition.LEFT;
                    movement++;   // Increments to next space
                    break;
                case 1: //land robot and adjust robot and get robot away from the lander, so it can collect minerals
                    telemetry.addData("case START: ", movement);
                    telemetry.update();
//                  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                  gravity = imu.getGravity();
                    //LOWER ROBOT DOWN TO PLAYING FIELD
                    myLiftMotor.extendLiftMotorFully(liftDistanceSensor);
                    sleep(sleepTime);
                    myMechDrive.strafeRight(SPD_DRIVE_MED,.5);
                    sleep(sleepTime);
                    //RETRACT LIFT ARM
                    myLiftMotor.retractLiftMotorFully(liftDistanceSensor);
                    sleep(sleepTime);
                    myMechDrive.driveForward(SPD_DRIVE_MED, .6); // move away from the lander toward crater
                    sleep(sleepTime);
                    myMechDrive.strafeLeft(SPD_DRIVE_MED, .6);
                    sleep(sleepTime);

                    // moving off lander
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
                    telemetry.addData("done with case: ", movement);
                    movement ++;  // increments to case 2
                    break;
                case 2:           // move to correct mineral / knock it off
                    telemetry.addData("case START ", movement);
                    telemetry.update();
                    sleep(1000);
                    switch (goldPosition) {
                        case LEFT: { //mineral left
                            telemetry.addLine("Left");
                            telemetry.update();
                            myMechDrive.strafeLeft(SPD_DRIVE_MED, 1.6);
                            sleep(sleepTime);
                            myMechDrive.driveForward(SPD_DRIVE_MED, .8);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, .55);
                            sleep(sleepTime);
                            myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.20);
                            sleep(sleepTime);
                            break;
                        }
                        case RIGHT: { //mineral right
                            telemetry.addLine("Right");
                            telemetry.update();
                            myMechDrive.strafeRight(SPD_DRIVE_MED, 1.4);
                            sleep(sleepTime);
                            myMechDrive.driveForward(SPD_DRIVE_MED, .8);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, .55);
                            sleep(sleepTime);

                            myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.2);
                            myMechDrive.driveForward(SPD_DRIVE_MED, 2);
                            break;
                        }
                        case MIDDLE: { // mineral straight
                            telemetry.addLine("Middle");
                            telemetry.update();
                            myMechDrive.driveForward(SPD_DRIVE_MED, .8);
                            sleep(sleepTime);
                            myMechDrive.driveBackward(SPD_DRIVE_MED, .55);
                            sleep(sleepTime);
                            myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.2);
                            sleep(sleepTime);
                            myMechDrive.driveForward(SPD_DRIVE_MED, 1);
                            sleep(sleepTime);
                            break;
                        }
                    }
                    sleep(100);
                    movement++;
                    break;
                case 3: //driving towards wall
                    myMechDrive.driveForward(SPD_DRIVE_MED, 2);
                    sleep(sleepTime);
//                    myMechDrive.rotateLeft(SPD_DRIVE_MED,.37);
                    //sleep(100);
                    myMechDrive.rotateLeft(SPD_DRIVE_MED, .57); //rorate at wall
                    sleep(sleepTime);
                    myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_MED);
                    sleep (2000); //orient self with wall
                    myMechDrive.stopMotors();
                    myMechDrive.driveForward(SPD_DRIVE_MED, 3.2 ); // 3.5
                    sleep(sleepTime);
                    myMechDrive.strafeLeft(SPD_DRIVE_MED,.7);
                    sleep(sleepTime);
                    myMechDrive.rotateRight(SPD_DRIVE_MED, 1.05);
                    sleep (sleepTime);


                    myTeamMarker.teamMarkerArmOutside();
                    sleep(1000);

                    myTeamMarker.teamMarkerArmRaised();

                    myMechDrive.rotateRight(SPD_DRIVE_MED,1.05);
                    sleep(sleepTime);
                    myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_MED);
                    sleep(2000); //go into the wall!
                    myMechDrive.stopMotors();
                    sleep(sleepTime);
                    myMechDrive.driveForward(SPD_DRIVE_MED,5);
                    sleep(sleepTime);

                    movement++;
                    break;

                case 4: // place team marker / servo arm
//                    myTeamMarker.teamMarkerArmLowered();
//                    sleep(100);
//                    myTeamMarker.teamMarkerArmRaised();
                    movement++;
                    break;

                case 5: // park in crater need to look at different pathways that we could take
                    //myMechDrive.driveBackward(SPD_DRIVE_MED, -3.2);
                    sleep(sleepTime);
                    movement++;
                    break;
//
//                case 6: //testing servo arm
//                    myTeamMarker.teamMarkerArmLowered();
//                    sleep(sleepTime);
//                    myTeamMarker.teamMarkerArmRaised();
//                    sleep(sleepTime);
//                    //        movement++;
//                    break;

            }

            idle();
//            requestOpModeStop();

        }

    }
}

package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerCrater {

    public MecanumMineralMinerCrater() {

    }

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .38;                  //Lowest speed
    final double SPD_DRIVE_MED = 0.6;   //was .4;                   Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .80;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 50;
    final int servoRotateTeamMarker = 1000;


    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }


    // new methods for this class

    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 200 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.RIGHT;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 200) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.MIDDLE;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.LEFT;
    }

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {

        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.strafeLeft(SPD_DRIVE_MED, .5);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .51);
        linearOp.sleep(sleepTime);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:

                linearOp.telemetry.update();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);

                linearOp.telemetry.update();
                myGyro.gyroOrientMecanum(34, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 0.7);              // fixing Gyro issue of over rotating

                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(-1, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.extendTowardMiddleMineral();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.3);              // fixing Gyro issue of over rotating
                linearOp.sleep(sleepTime);


                break;

            case RIGHT:
//                linearOp.telemetry.addData("RIGHT", goldPosition);
//                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-32, myMechDrive);          // Gyro angles appears correct. was -14
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeServo.IntakeServoForward();
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myIntakeServo.stopIntakeServo();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.9);              // fixing Gyro issue of over rotating was 1.9
                break;
        }
    }


    public void RotateDriveWall(GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm) {

        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED, .55);  // to get away from lander - too far = hit mineral, not enough = hit lander going to wall

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.2);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.3);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.4);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
        }


        linearOp.sleep(sleepTime);
    }


    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);                      // Align to wall
        linearOp.sleep(800);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam  was med
        linearOp.sleep(100);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(138, myMechDrive);              // Orient for straight drive to depot was 137
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, .7);

    }


    public void LowerReleaseTM(IntakeExtenderArm myIntakeExtenderArm, IntakeRotator myIntakeRotator, IntakeServo myIntakeServo) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.mineralRotateLowerEncoder();
        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.extendIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverse();
        linearOp.sleep(servoRotateTeamMarker);

        myIntakeExtenderArm.retractIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();
        myIntakeServo.stopIntakeServo();

        myIntakeRotator.mineralRotateRaiseEncoder();
        linearOp.sleep(sleepTime);
    }


    public void DriveParkInCrater(MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtender, IntakeRotator myIntakeRotater) {
        // drive backward and park in crater

        myMechDrive.driveBackward(1, .9);
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);       // was med
        linearOp.sleep(200);
        myMechDrive.stopMotors();

        myMechDrive.driveBackward(.35, .5);         // was 1.1
        linearOp.sleep(sleepTime);

        myMechDrive.driveBackward(.28, .4);
        linearOp.sleep(sleepTime);
    }

//    public void CraterExtendArm(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
//        myGyro.gyroReset();
//        myMechDrive.strafeLeft(SPD_DRIVE_HIGH,0.6);
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 0.4);
//        linearOp.sleep(sleepTime);
//        myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 3.1);
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(SPD_DRIVE_HIGH, 0.5);
//        linearOp.sleep(sleepTime);
//        myMechDrive.strafeRightCurvedToCrater(0.5, 0.45, 0.7, 0.75, 2.2);
//        myIntakeExtenderArm.extendHalfIntakeArmAuto();
//        myIntakeServo.IntakeServoForward();
//        myIntakeRotator.mineralRotateLowerEncoder();
//        myIntakeExtenderArm.extendHalfIntakeArmAuto();
//        linearOp.sleep(1000);
//    }
}

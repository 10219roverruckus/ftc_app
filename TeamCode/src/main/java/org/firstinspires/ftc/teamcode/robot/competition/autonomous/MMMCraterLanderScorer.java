package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MMMCraterLanderScorer {

    public MMMCraterLanderScorer() {

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


    public void driveTowardDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(50, myMechDrive);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2);
        myGyro.gyroOrientMecanum(74, myMechDrive);
        myIntakeExtenderArm.retractPowerAuto(1);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, 1);


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

    public void setUpForLander (GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myMechDrive.driveBackward(SPD_DRIVE_HIGH,1);
        myGyro.gyroOrientMecanum(74, myMechDrive);

        myMechDrive.driveBackward(SPD_DRIVE_HIGH, 2);
        myMechDrive.rotateRight(SPD_DRIVE_HIGH,.4);

        myGyro.gyroOrientMecanum(5, myMechDrive);
    }

    public void intakeGoldMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        switch (goldPosition) {
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

    public void

}

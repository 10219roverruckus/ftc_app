package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotaterServos;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeSpinnerMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerDepot {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .20;                  //Lowest speed
    final double SPD_DRIVE_MED = .5;                   //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 100;
    final int servoRotateTeamMarker = 2000;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MecanumMineralMinerDepot() {
    }


    // new Methods
    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 200 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.RIGHT;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 200) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.MIDDLE;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.MIDDLE;
    }


    public void driveMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotaterServos myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeSpinnerMotor myIntakeSpinnerMotor) {
        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
//        linearOp.sleep(3500);  //SIMULATE LOWING ROBOT - COMMENT OUT WHEN RUNNING LIFT MOTOR!
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(.3, .4);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_HIGH, .51);
        linearOp.sleep(sleepTime);

        switch (goldPosition) {
            case LEFT:

                myGyro.gyroOrientMecanum(-65, myMechDrive);               // different (34)
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-65, myMechDrive);               // different (34)
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();                     //lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(2400, SPD_DRIVE_HIGH, 3);                       // extend extender and knock off the mineral
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-90, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-90, myMechDrive);                  // corretion of angle with gyro
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(-1);                         // spit out the team marker
                linearOp.sleep(servoRotateTeamMarker);

                myGyro.gyroOrientMecanum(-35, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
                myMechDrive.stopMotors();
                myIntakeSpinnerMotor.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArm(0,SPD_DRIVE_HIGH,3);                     // retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(sleepTime);

                myIntakeRotator.raisedRotater();                    // raise rotater
                linearOp.sleep(sleepTime);

                break;


            case MIDDLE:
                myGyro.gyroOrientMecanum(-90, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeRotator.loweredRotater();                 //lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(2400, SPD_DRIVE_HIGH,3);             // extend extender and knock off the mineral
                linearOp.sleep(sleepTime);


                myIntakeSpinnerMotor.intakeSpinner(-1);                     // spit out the team marker
                linearOp.sleep(servoRotateTeamMarker);


                myIntakeExtenderArm.retractIntakeArm(0, SPD_DRIVE_HIGH, 3);                     // retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);            // another retract to pull the extender all the way back
                linearOp.sleep(1000);

                myIntakeExtenderArm.stopIntakeArm();
                myIntakeSpinnerMotor.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeRotator.raisedRotater();                    // raise rotater
                linearOp.sleep(sleepTime);
                break;

            case RIGHT:

                myGyro.gyroOrientMecanum(-120, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-120, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeRotator.loweredRotater();                    // lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArm(2400, SPD_DRIVE_HIGH,3);                      // extend extender to knock off the mineral
                linearOp.sleep(sleepTime);


                myGyro.gyroOrientMecanum(-90, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-90, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeSpinnerMotor.intakeSpinner(-1);                         // spit out mineral
                linearOp.sleep(servoRotateTeamMarker);


                myGyro.gyroOrientMecanum(-150, myMechDrive);
                myMechDrive.stopMotors();
                myIntakeSpinnerMotor.stopMotors();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArm(0, SPD_DRIVE_HIGH, 3);                     // retract extender arm
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(sleepTime);

                myIntakeRotator.raisedRotater();                    // raise the rotater
                linearOp.sleep(sleepTime);

                break;
        }
    }


    public void RotateDriveTowardCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myGyro.gyroOrientMecanum(-5, myMechDrive);                   // gyro towards the crater after dropping tm was -84.9
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(-5, myMechDrive);                   // gyro towards the crater after dropping tm was -84.9
        linearOp.sleep(sleepTime);


        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.5);               // drive toward wall in crater

    }

    public void DriveParkInCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm, IntakeRotaterServos myIntakeRotater) {

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(500);
        myIntakeExtenderArm.stopIntakeArm();
        myGyro.gyroOrientMecanum(46, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(46, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);


        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_MED);                      // Align to wall
        linearOp.sleep(1100);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam
        linearOp.sleep(150);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(46, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractIntakeArm(0, SPD_DRIVE_HIGH, 3);
        linearOp.sleep(sleepTime);

        myIntakeRotater.loweredRotater();
        linearOp.sleep(sleepTime);


    }


  }



package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.*;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControlled")
public class DriverControlled extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot      = new Robot(this);
    boolean slowmo   = false;
    boolean changed  = false; //Outside of loop()
    boolean shooting = false;
    boolean changed2 = false; //Outside of loop()
    double  speedreduction;
    double  ringtogglespeed;
    double  ringshooterspeed;

    @Override
    public void runOpMode() {
        // Initialize all the Robot Parts.
        // First Init the Drive Train by setting up the DriveTrain Motors and Sensors.
        telemetry.addData("Robot Initialized ... ", "START");    //
        telemetry.update();
        robot.getDriveTrain().init();
        robot.getWobbleGripper().init();
        robot.getRingControl().init();
        robot.getImuControl().init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot Initialized ... ", "DONE");    //
        robot.getImuControl().readimuheading();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // clear telemetry to clean up screen
        telemetry.clearAll();


        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Gamepad 1 ----------------------------------------------------------------------------

            //Navigation

            //Slow Motion Toggle
            if (slowmo == true) speedreduction = 0.2;
            else speedreduction = 0.6;
            if(gamepad1.start && !changed) {
                slowmo = !slowmo;
                changed = true;
            } else if(!gamepad1.start) changed = false;

            //Drive using speed reduction
            robot.getDriveTrain().DriverControlled_Drive(speedreduction);



            //Ring Intake Controls
            if (gamepad1.a)ringtogglespeed = 1.0;
            else if (gamepad1.y) ringtogglespeed = -1.0;
            else if (gamepad1.back) ringtogglespeed = 0.0;
            robot.getRingControl().DriverControlledRingIntake(ringtogglespeed);




            //Bucket Servo Controls TEMP*******************
            double YEETFORCE_SET = 0.9;




            //Gamepad 2 ----------------------------------------------------------------------------

            //Gripper servo to grab or drop the wobble goal
            if (gamepad2.a) robot.getWobbleGripper().setWobbleGripper(0.7);
            else robot.getWobbleGripper().setWobbleGripper(0.0);



            //Firing

            //Toggle engage the constant ring shooter
            if(gamepad2.x && !changed2) {
                shooting = !shooting;
                changed2 = true;
            } else if(!gamepad2.x) changed2 = false;

            //Normal fire speed
            if (shooting) robot.getRingControl().ConstantRingShooter(YEETFORCE_SET);
            else robot.getRingControl().ConstantRingShooter(0);

            //Dampened fire speed
            if (gamepad2.left_trigger != 0){
                robot.getRingControl().ConstantRingShooter(0.7);
             }
            else if (gamepad2.left_trigger == 0 && shooting) {
                robot.getRingControl().ConstantRingShooter(YEETFORCE_SET);
            }




            //Move the wobble goal arm
            if (gamepad2.left_stick_y != 0) {
                robot.getWobbleGripper().setArmPower(gamepad2.left_stick_y * 0.75);
            }
            else robot.getWobbleGripper().setArmPower(0.0);




            //Activates the servo to push the ring into the shooter, FIRING!!!!!!!!
            if (gamepad2.right_trigger != 0){
                robot.getRingControl().DriverControlledRingShooter(0.24);
            }
            else robot.getRingControl().DriverControlledRingShooter(0.5);




            telemetry.update();
            telemetry.clearAll();
        // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }

}


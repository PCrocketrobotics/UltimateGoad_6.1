
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;

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

@TeleOp(name="Driver_Controlled_Basic")
public class Driver_Controlled_Basic extends LinearOpMode {

    static final int            COUNTS_PER_MOTOR_REV   = 28;      // Motor with 1:1 gear ratio
    static final double         DRIVE_GEAR_REDUCTION   = 10.5;    // Rev Ultraplanetary Motor 12:1 but actual is 10.5:1
    static final double         WHEEL_DIAMETER_INCHES  = 3.0;     // For figuring circumference
    static final double         COUNTS_PER_INCH_DOUBLE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotorEx left_front;
    public DcMotorEx left_back;
    public DcMotorEx right_front;
    public DcMotorEx right_back;

    /* Declare OpMode members. */
    Robot robot      = new Robot(this);

    @Override
    public void runOpMode() {
        // Initialize all the Robot Parts.
        // First Init the Drive Train by setting up the DriveTrain Motors and Sensors.
        telemetry.addData("Robot Initialized ... ", "START");    //
        telemetry.update();
        robot.getDriveTrain().init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot Initialized ... ", "DONE");    //
        robot.getImuControl().readimuheading();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Clear telemetry to clean up screen.
        telemetry.clearAll();


        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Gamepad 1 ----------------------------------------------------------------------------

            telemetry.update();
            telemetry.clearAll();
        // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }

}


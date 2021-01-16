package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Red", group="Production")

public class Autonomous_Red extends LinearOpMode {
    Robot robot = new Robot (this);
    int Target_Zone;
    boolean run;

    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }
    @Override
    public void runOpMode(){
        String ringpattern = "";
        robot.getDriveTrain().init();
        robot.getComputerVision().init();
        robot.getImuControl().init();
        sleep(500);
        
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {

        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ringpattern = robot.getComputerVision().detect();
                sleep(500);

                //looks at the amount of rings and returns a value based on how many it sees
                if (ringpattern == "None") {
                    Target_Zone = 0 ;
                }
                else if (ringpattern == "Single") {
                    Target_Zone = 1 ;
                }
                else if (ringpattern == "Quad") {
                    Target_Zone = 4 ;
                }
                telemetry.addData("ring patter", ringpattern);
                telemetry.addData("TargetZone", Target_Zone);
                telemetry.addData("", "----------------------------");
                telemetry.addData("sec", String.format("%.2f", timer.seconds()));

                telemetry.addData(">", "Press Play to Start");
                telemetry.update();
                sleep(1400);
                if (Target_Zone == 0) {
                    break;
                }
                else if (Target_Zone == 1) {
                    break;
                }
                else if (Target_Zone == 4) {
                    break;
                }

            }
            robot.getRingControl().DriverControlledRingShooter(.5);
            if (Target_Zone == 0) {
                robot.getDriveTrain().gyroDrive(30, 30, 30, 30, 0.4, 1);
                robot.getDriveTrain().moveToColor("white", 0.2);
                //robot.getDriveTrain().gyroDrive(); //Strafe to the left.
                //robot.getDriveTrain().gyroDrive(); //Back up behind white line.
                //robot.getDriveTrain().gyroTurn(0.15, 180); //Spin 180 degrees.
                //robot.getRingControl().ConstantRingShooter(0.7); //Set speed of shooters where necessary.
                //robot.getDriveTrain().gyroTurn(); //Turn degrees to aim at 1st powershot.
                //robot.getRingControl().DriverControlledRingShooter(0.24);
                //robot.getRingControl().DriverControlledRingShooter(0.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 2nd powershot.
                //robot.getRingControl().DriverControlledRingShooter(0.24);
                //robot.getRingControl().DriverControlledRingShooter(0.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 3rd powershot.
                //robot.getRingControl().DriverControlledRingShooter(0.24);
                //robot.getRingControl().DriverControlledRingShooter(0.5);
                //robot.getDriveTrain().gyroTurn(0.3, 180); //Spin back to course.
                //robot.getDriveTrain().moveToColor("white", 0.3); //Move back to white.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move wobble goal into position.
                //robot.getWobbleGripper().setWobbleGripper(); //Release wobble goal.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move arm back to original position.
                //robot.getRingControl().ConstantRingShooter(0); //Set speed of motors to 0.

            }
            if (Target_Zone == 1) {
                robot.getDriveTrain().moveToColor("white", .2);
                //robot.getDriveTrain().gyroDrive(); //Strafe to the left.
                //robot.getDriveTrain().gyroTurn(); //Pivot 90 degrees to face the spot.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move wobble goal into position.
                //robot.getWobbleGripper().setWobbleGripper(); //Release wobble goal.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move arm back to original position.
                //robot.getDriveTrain().gyroTurn(); //Pivot 90 degrees forward.
                //robot.getDriveTrain().gyroDrive(); //Back up slightly behind white line.
                //robot.getRingControl().ConstantRingShooter(0.7); //Set speed of motors.
                //robot.getDriveTrain().gyroTurn(); //Turn degrees to aim at 1st powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 2nd powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 3rd powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().moveToColor("white", -0.3); //Move back to white.
                //robot.getRingControl().ConstantRingShooter(0); //Set speed of motors to 0.

            }
            if (Target_Zone == 4) {
                robot.getDriveTrain().moveToColor("white", .2);
                //robot.getDriveTrain().gyroDrive(); //Strafe to the left.
                //robot.getDriveTrain().gyroDrive(); //Back up behind white line.
                //robot.getDriveTrain().gyroTurn(0.3, 180); //Spin 180 degrees.
                //robot.getRingControl().ConstantRingShooter(); //Set speed of motors.
                //robot.getDriveTrain().gyroTurn(); //Turn degrees to aim at 1st powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 2nd powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().gyroTurn(); //Turn to aim at 3rd powershot.
                //robot.getRingControl().DriverControlledRingShooter(.24);
                //robot.getRingControl().DriverControlledRingShooter(.5);
                //robot.getDriveTrain().gyroTurn(0.3, 180); //Spin 180 degrees.
                //robot.getDriveTrain().moveToColor("red", .2); //Continue to move to red.
                //robot.getDriveTrain().gyroDrive(3, 3, 3, 3, .2, 1); //Drive past the first red checkpoint.
                //robot.getDriveTrain().moveToColor("red", .2); //Move to the last red spot.
                //robot.getDriveTrain().gyroDrive(3, 3, 3, 3, .2, 1); //Move a slight bit forward to be even with the last drop off point.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move wobble goal into position.
                //robot.getWobbleGripper().setWobbleGripper(); //Release wobble goal.
                //robot.getWobbleGripper().autonomousArmMovement(); //Move arm back to original position.
                //robot.getDriveTrain().moveToColor("white", -0.2); //Move back to white.
                //robot.getRingControl().ConstantRingShooter(0); //Set speed of motors to 0.

            }


        }

    }
}

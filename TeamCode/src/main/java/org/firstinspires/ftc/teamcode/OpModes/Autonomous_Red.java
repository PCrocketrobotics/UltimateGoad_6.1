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
        robot.getRingControl().init();
        sleep(500);
        
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {

        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robot.getRingControl().DriverControlledRingShooter(0.5);
                robot.getDriveTrain().gyroDrive(-6, -6, -6, -6, 0.2, 1);
                robot.getDriveTrain().gyroTurn(0.2, 18);
                ringpattern = robot.getComputerVision().detect();
                sleep(500);
                ringpattern = robot.getComputerVision().detect();
                sleep(500);
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
                sleep(2000);
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
            robot.getDriveTrain().gyroTurn(0.2, 0);
            robot.getDriveTrain().gyroDrive(-42, -42, -42, -42, .3, 1);
            robot.getDriveTrain().gyroTurn(0.2, 9);
            sleep(500);
            robot.getRingControl().ConstantRingShooter(0.9);
            sleep(1000);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().ConstantRingShooter(0);
            robot.getDriveTrain().gyroTurn(.2, 0);

            if (Target_Zone == 0) {
                robot.getDriveTrain().moveToColor("white", -0.2);

            }
            if (Target_Zone == 1) {
                robot.getDriveTrain().moveToColor("white", -0.2);
                robot.getDriveTrain().gyroTurn(.2, 90);
                robot.getDriveTrain().gyroDrive(-12, -12, -12, -12, 0.3, 1);
                robot.getDriveTrain().gyroDrive(-8, 8, 8, -8, 0.3, 1);
                sleep(1000);
                robot.getDriveTrain().gyroDrive(8, -8, -8, 8, 0.3, 1);

            }
            if (Target_Zone == 4) {
                robot.getDriveTrain().moveToColor("white", -0.2);
                robot.getDriveTrain().moveToColor("red", -0.2);
                robot.getDriveTrain().gyroDrive(-3, -3, -3, -3, 0.2, 1);
                robot.getDriveTrain().moveToColor("red", -0.2);
                robot.getDriveTrain().gyroDrive(-5, -5, -5, -5, .2, 1);
                robot.getDriveTrain().moveToColor("white", 0.2);
                
            }




        }

    }
}
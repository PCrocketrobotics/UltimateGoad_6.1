package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Blue 2 Powershot", group="Production")

public class Autonomous_Blue_2_Powershot extends LinearOpMode {
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
        robot.getWobbleGripper().init();
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
            robot.getDriveTrain().moveToColor("white", -0.2);
            robot.getDriveTrain().gyroDrive(12.5, 12.5, 12.5, 12.5, 0.3, 1);
            robot.getRingControl().ConstantRingShooter(0.88);
            robot.getDriveTrain().gyroTurn(0.1, 0);
            robot.getDriveTrain().gyroDrive(-21.5, 21.5, 21.5, -21.5, 0.3, 1);
            robot.getDriveTrain().gyroTurn(0.1, 0);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getDriveTrain().gyroDrive(8, -8, -8, 8, 0.3, 1);
            robot.getDriveTrain().gyroTurn(0.1, 0);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);
            robot.getRingControl().ConstantRingShooter(0.86);
            robot.getDriveTrain().gyroDrive(8, -8, -8, 8, 0.3, 1);
            robot.getDriveTrain().gyroTurn(0.1, 0);
            robot.getRingControl().DriverControlledRingShooter(0.24);
            sleep(500);
            robot.getRingControl().DriverControlledRingShooter(0.5);
            sleep(500);

            if (Target_Zone == 0) {
                robot.getDriveTrain().moveToColor("white", -0.2);
                robot.getDriveTrain().gyroDrive(20, -20, -20, 20, 0.3, 1);
                robot.getDriveTrain().gyroTurn(0.3, 180);
                robot.getWobbleGripper().autonomousArmMovement(-0.5);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getWobbleGripper().setWobbleGripper(0.7);
                robot.getWobbleGripper().autonomousArmMovement(0.75);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getDriveTrain().gyroDrive(15, -15, -15, 15, 0.3, 1);


            }
            if (Target_Zone == 1) {
                robot.getDriveTrain().gyroDrive(-39, -39, -39, -39, 0.3, 1);
                robot.getDriveTrain().gyroTurn(0.3, 180);
                robot.getWobbleGripper().autonomousArmMovement(-0.5);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getWobbleGripper().setWobbleGripper(0.7);
                robot.getWobbleGripper().autonomousArmMovement(0.75);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getDriveTrain().moveToColor("white", -0.22);

            }
            if (Target_Zone == 4) {
                robot.getDriveTrain().gyroDrive(-50, -50, -50, -50, 0.3, 1);
                robot.getDriveTrain().gyroDrive(30, -30, -30, 30, 0.3, 1);
                robot.getDriveTrain().gyroTurn(0.3, 180);
                robot.getWobbleGripper().autonomousArmMovement(-0.5);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getWobbleGripper().setWobbleGripper(0.7);
                robot.getWobbleGripper().autonomousArmMovement(0.75);
                sleep(2000);
                robot.getWobbleGripper().autonomousArmMovement(0);
                robot.getDriveTrain().gyroTurn(0.3, -150);
                robot.getDriveTrain().gyroDrive(65, 65, 65, 65, 0.3, 1);
                
            }




        }

    }
}

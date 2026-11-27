package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.CameraLL;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

/**
 * Autonomous Action Helper Class modeling RoadRunner Actions for Shooter, Intake, and Drive.
 */
public class SSMyRobot {

    private final Shooter shooter;
    private final Intake intake;
    private final TankDrive drive;
    private final HardwareMap myHardwareMap;
    public final FtcDashboard dashboard;

    private final CameraLL camera;
    public ElapsedTime generalTimer = new ElapsedTime();

    public SSMyRobot(HardwareMap hardwareMap, TankDrive drive, Intake intake, Shooter shooter, Pose2d startPose, CameraLL camera) {
        this.myHardwareMap = hardwareMap;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.dashboard = FtcDashboard.getInstance();
        this.generalTimer.reset();
        this.camera = camera;

        if (this.drive != null && startPose != null) {
            this.drive.localizer.setPose(startPose);
        }
    }

    // ==========================================
    // SHOOTER ACTIONS
    // ==========================================

    public class SpinFlywheels implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.spin();
            return false;
        }
    }
    public Action spinFlywheels() { return new SpinFlywheels(); }

    public class StopFlywheels implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.stop();
            return false;
        }
    }
    public Action stopFlywheels() { return new StopFlywheels(); }

    public class AutoSetShooterFromDistance implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.autoSetShooterFromDistance();
            return false;
        }
    }
    public Action autoSetShooterFromDistance() { return new AutoSetShooterFromDistance(); }

    public class SetHiveTarget implements Action {
        private final Shooter.HiveTarget target;

        public SetHiveTarget(Shooter.HiveTarget target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.setHiveTarget(target);
            return false;
        }
    }
    public Action setHiveTarget(Shooter.HiveTarget target) { return new SetHiveTarget(target); }

    public class CycleRedHives implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.cycleRedHives();
            return false;
        }
    }
    public Action cycleRedHives() { return new CycleRedHives(); }

    public class CycleBlueHives implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.cycleBlueHives();
            return false;
        }
    }
    public Action cycleBlueHives() { return new CycleBlueHives(); }

    public class AlignToHive implements Action {
        private boolean firstRun = true;
        private final Shooter.HiveTarget target;

        public AlignToHive(Shooter.HiveTarget target) {
            this.target = target;
        }

        public AlignToHive() {
            this.target = shooter.getSelectedHiveTarget();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            if (firstRun) {
                shooter.updateHiveAutoAlign(true, target);
                firstRun = false;
                return true;
            }
            return shooter.updateHiveAutoAlign(false, target);
        }
    }
    public Action alignToHive(Shooter.HiveTarget target) { return new AlignToHive(target); }
    public Action alignToHive() { return new AlignToHive(); }

    // ==========================================
    // INTAKE ACTIONS
    // ==========================================

    public class SetIntakePower implements Action {
        private final double power;

        public SetIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.intakeMotor.setPower(power);
            intake.left.setPower(-power);
            intake.right.setPower(power);
            return false;
        }
    }
    public Action setIntakePower(double power) { return new SetIntakePower(power); }

    public class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.gate.setPosition(0.3);
            return false;
        }
    }
    public Action openGate() { return new OpenGate(); }

    public class CloseGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.gate.setPosition(0.4);
            return false;
        }
    }
    public Action closeGate() { return new CloseGate(); }





    public class CloseNectarGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.nectarGate.setPosition(1.0);
            return false;
        }
    }
    public Action closeNectarGate() { return new CloseNectarGate(); }
    // ==========================================
    // ROBOT UPDATE ACTION
    // ==========================================

    public class UpdateRobot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket pack) {
            shooter.update();
            pack.put("Hive Target", shooter.getSelectedHiveTarget().name);
            pack.put("Hive Distance", shooter.getHiveDistance());
            pack.put("Heading Offset Deg", shooter.getLastHeadingOffsetDeg());
            pack.put("Hive Aligned", shooter.isHiveAligned());
            return true;
        }
    }
    public Action updateRobot() { return new UpdateRobot(); }
}
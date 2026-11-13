package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz HIVE AUTO BLUE", group = "BioBuzz", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzHiveAutoBlue extends BioBuzzHiveAutoBase {
    @Override
    protected AutoField.Alliance alliance() {
        return AutoField.Alliance.BLUE;
    }
}

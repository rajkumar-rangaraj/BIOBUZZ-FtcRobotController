package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Disabled   // v1 (windmill robot): parked 21 Nov 2026, see v2/

@Autonomous(name = "v1 OLD AUTO BLUE", group = "BioBuzz Old", preselectTeleOp = "v1 BioBuzz TeleOp")
public class BioBuzzAutoBlue extends BioBuzzAutoBase {
    @Override
    protected HiveVision.Alliance alliance() {
        return HiveVision.Alliance.BLUE;
    }
}

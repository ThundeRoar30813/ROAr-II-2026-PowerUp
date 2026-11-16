package org.firstinspires.ftc.teamcode.Autonomos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class FieldRoutes {

    private static final PoseFactory pf = PoseFactory.degrees();

    // =========================================
    // POSICIONES FISICAS EN CANCHA
    // =========================================
    public static final Pose START_ROJO_DER   = pf.of(44.5916, 9.0788, 90);
    public static final Pose TIRO_INICIAL     = pf.of(44.6143, 15.2436, 90);

    public static final Pose PRE_FLOR_1       = pf.of(17.8901, 46.9881, 180);
    public static final Pose FLOR_1           = pf.of(9.9444, 46.7875, 180);

    public static final Pose PRE_FLOR_2       = pf.of(47.38, 125.9494, 90);
    public static final Pose FLOR_2           = pf.of(47.047, 131.2394, 90);

    public static final Pose PARK_ROJO        = pf.of(9.8474, 120.0793, 90);

    // =========================================
    // TRAYECTORIAS (Origen_a_Destino)
    // =========================================

    public static Path start_a_tiroInicial() {
        return Paths.line(START_ROJO_DER, TIRO_INICIAL)
                .constant(TIRO_INICIAL);
    }

    public static Path tiroInicial_a_preFlor1() {
        return Paths.curve(
                TIRO_INICIAL,
                pf.of(41.4224, 34.4952, 0),
                pf.of(37.5983, 44.159, 0),
                PRE_FLOR_1
        ).linear(TIRO_INICIAL, PRE_FLOR_1);
    }

    public static Path preFlor1_a_flor1() {
        return Paths.line(PRE_FLOR_1, FLOR_1)
                .constant(FLOR_1);
    }

    public static Path flor1_a_preFlor1() {
        return Paths.line(FLOR_1, PRE_FLOR_1)
                .constant(PRE_FLOR_1);
    }

    public static Path preFlor1_a_preFlor2() {
        return Paths.curve(
                PRE_FLOR_1,
                pf.of(30.5411, 111.1691, 0),
                PRE_FLOR_2
        ).linear(PRE_FLOR_1, PRE_FLOR_2);
    }

    public static Path preFlor2_a_flor2() {
        return Paths.line(PRE_FLOR_2, FLOR_2)
                .constant(FLOR_2);
    }

    public static Path flor2_a_preFlor2() {
        return Paths.line(FLOR_2, PRE_FLOR_2)
                .constant(PRE_FLOR_2);
    }

    public static Path preFlor2_a_park() {
        return Paths.line(PRE_FLOR_2, PARK_ROJO)
                .constant(PARK_ROJO);
    }

    // Ruta extra por si en el Regional tu alianza pide hacer solo 1 flor + park
    public static Path preFlor1_a_park() {
        return Paths.line(PRE_FLOR_1, PARK_ROJO)
                .constant(PARK_ROJO);
    }
}
// OpenRocket reference harness for Bowshock.
//
// Loads an .ork with OpenRocket's own loader and dumps, as JSON, everything the Bowshock comparison needs:
//   * static aerodynamics (BarrowmanCalculator) at alpha = 0 and 2 deg over Mach 0.1..2.0 step 0.05,
//   * launch / burnout mass and CG (MassCalculator),
//   * flight-simulation summaries (stored simulations, "as stored" and "calm" = no wind/turbulence),
//   * the motors used, exported as RASP .eng files.
//
// Run:  java -Djava.awt.headless=true -cp OpenRocket.jar Harness.java <file.ork> <outdir>
// See tools/openrocket-harness/README.md.

import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.inject.Guice;
import com.google.inject.Injector;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.BarrowmanCalculator;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.file.GeneralRocketLoader;
import net.sf.openrocket.logging.WarningSet;
import net.sf.openrocket.masscalc.MassCalculator;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.models.atmosphere.AtmosphericConditions;
import net.sf.openrocket.motor.Motor;
import net.sf.openrocket.motor.MotorConfiguration;
import net.sf.openrocket.motor.ThrustCurveMotor;
import net.sf.openrocket.plugin.PluginModule;
import net.sf.openrocket.rocketcomponent.FlightConfiguration;
import net.sf.openrocket.rocketcomponent.FlightConfigurationId;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.rocketcomponent.RocketComponent;
import net.sf.openrocket.simulation.FlightData;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.FlightEvent;
import net.sf.openrocket.simulation.SimulationOptions;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.startup.GuiModule;
import net.sf.openrocket.util.Coordinate;

public class Harness {
    static final double DEG = Math.PI / 180.0;

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("usage: Harness <file.ork> <outdir>");
            System.exit(2);
        }
        File ork = new File(args[0]);
        File outDir = new File(args[1]);
        outDir.mkdirs();
        File engDir = new File(outDir, "motors");
        engDir.mkdirs();

        // Same wiring as OpenRocket's SwingStartup, minus the GUI.
        GuiModule guiModule = new GuiModule();
        Injector injector = Guice.createInjector(guiModule, new PluginModule());
        Application.setInjector(injector);
        guiModule.startLoader();
        // Wait for the asynchronous motor database load ourselves; the blocking provider would otherwise
        // try to open a (non-headless) progress dialog.
        java.lang.reflect.Field lf = GuiModule.class.getDeclaredField("motorLoader");
        lf.setAccessible(true);
        ((net.sf.openrocket.database.MotorDatabaseLoader) lf.get(guiModule)).blockUntilLoaded();

        GeneralRocketLoader loader = new GeneralRocketLoader(ork);
        OpenRocketDocument doc = loader.load();
        Rocket rocket = doc.getRocket();

        Map<String, Object> root = new LinkedHashMap<>();
        root.put("file", ork.getName());
        root.put("rocket", rocket.getName());
        root.put("openrocket_version", net.sf.openrocket.util.BuildProperties.getVersion());
        List<String> loadWarnings = new ArrayList<>();
        for (net.sf.openrocket.logging.Warning w : loader.getWarnings()) {
            loadWarnings.add(w.toString());
        }
        root.put("load_warnings", loadWarnings);
        root.put("stage_count", rocket.getStageCount());
        root.put("perfect_finish", rocket.isPerfectFinish());

        // Component inventory (for diagnosing importer mapping).
        List<Object> comps = new ArrayList<>();
        walk(rocket, 0, comps);
        root.put("components", comps);

        List<Object> configs = new ArrayList<>();
        List<Object> sims = new ArrayList<>();
        List<FlightConfigurationId> ids = rocket.getIds();
        for (FlightConfigurationId id : ids) {
            FlightConfiguration fc = rocket.getFlightConfiguration(id);
            rocket.setSelectedConfiguration(id);
            fc.setAllStages();
            configs.add(dumpConfig(rocket, fc, engDir, ork.getName()));
        }
        root.put("configs", configs);

        // Simulations: stored ones, or a default one per configuration with motors.
        List<Simulation> toRun = new ArrayList<>(doc.getSimulations());
        if (toRun.isEmpty()) {
            for (FlightConfigurationId id : ids) {
                if (!rocket.hasMotors(id)) continue;
                Simulation s = new Simulation(doc, rocket);
                s.setFlightConfigurationId(id);
                s.setName("default " + id.toShortKey());
                toRun.add(s);
            }
        }
        for (Simulation sim : toRun) {
            for (boolean calm : new boolean[] {false, true}) {
                sims.add(runSim(rocket, sim, calm));
            }
        }
        root.put("simulations", sims);

        String stem = ork.getName().replaceAll("\\.ork$", "");
        File out = new File(outDir, stem + ".json");
        try (PrintWriter w = new PrintWriter(out, StandardCharsets.UTF_8)) {
            w.print(Json.write(root));
        }
        System.out.println("wrote " + out);
        System.exit(0);
    }

    static void walk(RocketComponent c, int depth, List<Object> out) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("depth", depth);
        m.put("type", c.getComponentName());
        m.put("name", c.getName());
        m.put("instances", c.getInstanceCount());
        try {
            m.put("mass_kg", c.getComponentMass());
            m.put("cg_x", c.getComponentCG().x);
            Coordinate[] loc = c.getComponentLocations();
            m.put("abs_x", loc.length > 0 ? loc[0].x : null);
            List<Object> xs = new ArrayList<>();
            for (int k = 0; k < Math.min(loc.length, 32); k++) xs.add(loc[k].x);
            m.put("locations_x", xs);
            m.put("length", c.getLength());
        } catch (Exception e) {
            m.put("error", e.toString());
        }
        m.put("mass_overridden", c.isMassOverridden());
        m.put("override_mass", c.getOverrideMass());
        m.put("override_sub_mass", c.isSubcomponentsOverriddenMass());
        m.put("cg_overridden", c.isCGOverridden());
        m.put("override_cg_x", c.getOverrideCGX());
        m.put("cd_overridden", c.isCDOverridden());
        try {
            m.put("eff_mass_kg", c.getMass());
            m.put("eff_cg_x", c.getCG().x);
        } catch (Exception e) {
            m.put("error2", e.toString());
        }
        out.add(m);
        for (RocketComponent ch : c.getChildren()) walk(ch, depth + 1, out);
    }

    static Map<String, Object> dumpConfig(Rocket rocket, FlightConfiguration fc, File engDir, String orkName)
            throws Exception {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("id", fc.getId().toShortKey());
        m.put("name", fc.getName());
        m.put("reference_length", fc.getReferenceLength());
        m.put("reference_area", fc.getReferenceArea());
        m.put("length", fc.getLength());
        m.put("aero_length", fc.getLengthAerodynamic());
        m.put("active_stages", fc.getActiveStageCount());

        List<Object> motors = new ArrayList<>();
        for (MotorConfiguration mc : fc.getAllMotors()) {
            Motor mo = mc.getMotor();
            if (mo == null) continue;
            Map<String, Object> mm = new LinkedHashMap<>();
            mm.put("designation", mo.getDesignation());
            mm.put("mount", mc.getMount().toString());
            mm.put("mount_stage", mc.getMount().getStage().getName());
            mm.put("count", mc.getMount().getInstanceCount());
            mm.put("ejection_delay_s", mc.getEjectionDelay());
            mm.put("ignition_event", mc.getIgnitionEvent().toString());
            mm.put("ignition_delay_s", mc.getIgnitionDelay());
            mm.put("position_x", mc.getPosition().x);
            mm.put("launch_mass_kg", mo.getLaunchMass());
            mm.put("burnout_mass_kg", mo.getBurnoutMass());
            mm.put("total_impulse_Ns", mo.getTotalImpulseEstimate());
            mm.put("burn_time_s", mo.getBurnTimeEstimate());
            if (mo instanceof ThrustCurveMotor) {
                String fname = writeEng((ThrustCurveMotor) mo, mc.getEjectionDelay(), engDir);
                mm.put("eng_file", "motors/" + fname);
            }
            motors.add(mm);
        }
        m.put("motors", motors);

        RigidBody launch = MassCalculator.calculateLaunch(fc);
        RigidBody burnout = MassCalculator.calculateBurnout(fc);
        RigidBody structure = MassCalculator.calculateStructure(fc);
        m.put("launch_mass_kg", launch.getMass());
        m.put("launch_cg_x", launch.cm.x);
        m.put("launch_Ixx", launch.getIxx());
        m.put("launch_Iyy", launch.getIyy());
        m.put("burnout_mass_kg", burnout.getMass());
        m.put("burnout_cg_x", burnout.cm.x);
        m.put("structure_mass_kg", structure.getMass());
        m.put("structure_cg_x", structure.cm.x);

        // Static aero sweep, ISA sea level.
        List<Object> aero = new ArrayList<>();
        BarrowmanCalculator calc = new BarrowmanCalculator();
        for (double alphaDeg : new double[] {0.0, 2.0}) {
            for (int i = 2; i <= 40; i++) {
                double mach = i * 0.05;
                if (i == 2) mach = 0.1;
                FlightConditions fcond = new FlightConditions(fc);
                fcond.setAtmosphericConditions(new AtmosphericConditions());
                fcond.setMach(mach);
                fcond.setAOA(alphaDeg * DEG);
                fcond.setRollRate(0);
                WarningSet ws = new WarningSet();
                AerodynamicForces f = calc.getAerodynamicForces(fc, fcond, ws);
                Map<String, Object> a = new LinkedHashMap<>();
                a.put("mach", mach);
                a.put("alpha_deg", alphaDeg);
                a.put("cp_x", f.getCP().x);
                a.put("cna", f.getCNa());
                a.put("cn", f.getCN());
                a.put("cd", f.getCD());
                a.put("cd_axial", f.getCDaxial());
                a.put("cd_friction", f.getFrictionCD());
                a.put("cd_pressure", f.getPressureCD());
                a.put("cd_base", f.getBaseCD());
                a.put("cm", f.getCm());
                aero.add(a);
            }
        }
        m.put("aero", aero);
        // Per-component forces (alpha = 0) at a few Mach numbers, to see where a total comes from.
        List<Object> byComp = new ArrayList<>();
        for (double mach : new double[] {0.3, 0.9, 1.0, 1.2, 2.0}) {
            FlightConditions fcond = new FlightConditions(fc);
            fcond.setAtmosphericConditions(new AtmosphericConditions());
            fcond.setMach(mach);
            fcond.setAOA(0);
            fcond.setRollRate(0);
            Map<RocketComponent, AerodynamicForces> forces = calc.getForceAnalysis(fc, fcond, new WarningSet());
            for (Map.Entry<RocketComponent, AerodynamicForces> e : forces.entrySet()) {
                AerodynamicForces f = e.getValue();
                Map<String, Object> a = new LinkedHashMap<>();
                a.put("mach", mach);
                a.put("type", e.getKey().getComponentName());
                a.put("name", e.getKey().getName());
                a.put("cna", f.getCNa());
                a.put("cp_x", f.getCP() == null ? null : f.getCP().x);
                a.put("cd", f.getCD());
                a.put("cd_friction", f.getFrictionCD());
                a.put("cd_pressure", f.getPressureCD());
                a.put("cd_base", f.getBaseCD());
                byComp.add(a);
            }
        }
        m.put("aero_components", byComp);
        return m;
    }

    /** Writes a RASP .eng for the motor with the chosen ejection delay; returns the file name. */
    static String writeEng(ThrustCurveMotor mo, double delay, File dir) throws Exception {
        String man = mo.getManufacturer().getSimpleName();
        String des = mo.getDesignation();
        String digest = mo.getDigest();
        String name = (man + "_" + des + "_" + digest.substring(0, Math.min(8, digest.length())) + "_d"
                + (Double.isInfinite(delay) ? "P" : String.valueOf((int) Math.round(delay))))
                .replaceAll("[^A-Za-z0-9_.-]", "_") + ".eng";
        StringBuilder sb = new StringBuilder();
        sb.append("; ").append(man).append(' ').append(des).append(" exported from OpenRocket thrust-curve database\n");
        String delayStr = Double.isInfinite(delay) || delay > 900 ? "P" : String.valueOf((int) Math.round(delay));
        sb.append(String.format(Locale.ROOT, "%s %.1f %.1f %s %.6f %.6f %s\n", des.replace(' ', '_'),
                mo.getDiameter() * 1000, mo.getLength() * 1000, delayStr, mo.getPropellantMass(),
                mo.getLaunchMass(), man.replace(' ', '_')));
        double[] t = mo.getTimePoints();
        double[] f = mo.getThrustPoints();
        for (int i = 0; i < t.length; i++) {
            if (t[i] <= 0) continue; // RASP data starts after t = 0
            sb.append(String.format(Locale.ROOT, "   %.5f %.5f\n", t[i], f[i]));
        }
        Files.writeString(new File(dir, name).toPath(), sb.toString());
        return name;
    }

    static Map<String, Object> runSim(Rocket rocket, Simulation sim, boolean calm) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("name", sim.getName());
        m.put("variant", calm ? "calm" : "stored");
        m.put("config", sim.getFlightConfigurationId().toShortKey());
        SimulationOptions o = sim.getOptions();
        double savedWind = o.getWindSpeedAverage();
        double savedTurb = o.getWindTurbulenceIntensity();
        double savedDev = o.getWindSpeedDeviation();
        if (calm) {
            o.setWindSpeedAverage(0);
            o.setWindTurbulenceIntensity(0);
        }
        Map<String, Object> c = new LinkedHashMap<>();
        c.put("rod_length_m", o.getLaunchRodLength());
        c.put("rod_angle_deg", o.getLaunchRodAngle() / DEG);
        c.put("rod_direction_deg", o.getLaunchRodDirection() / DEG);
        c.put("launch_into_wind", o.getLaunchIntoWind());
        c.put("wind_avg_ms", o.getWindSpeedAverage());
        c.put("wind_turbulence", o.getWindTurbulenceIntensity());
        c.put("wind_direction_deg", o.getWindDirection() / DEG);
        c.put("altitude_m", o.getLaunchAltitude());
        c.put("latitude_deg", o.getLaunchLatitude());
        c.put("isa", o.isISAAtmosphere());
        c.put("temperature_K", o.getLaunchTemperature());
        c.put("pressure_Pa", o.getLaunchPressure());
        c.put("time_step_s", o.getTimeStep());
        m.put("conditions", c);
        try {
            sim.simulate();
            FlightData d = sim.getSimulatedData();
            Map<String, Object> r = new LinkedHashMap<>();
            r.put("apogee_m", d.getMaxAltitude());
            r.put("max_velocity_ms", d.getMaxVelocity());
            r.put("max_mach", d.getMaxMachNumber());
            r.put("max_acceleration_ms2", d.getMaxAcceleration());
            r.put("time_to_apogee_s", d.getTimeToApogee());
            r.put("flight_time_s", d.getFlightTime());
            r.put("rail_exit_velocity_ms", d.getLaunchRodVelocity());
            r.put("ground_hit_velocity_ms", d.getGroundHitVelocity());
            r.put("deployment_velocity_ms", d.getDeploymentVelocity());
            r.put("branches", d.getBranchCount());
            FlightDataBranch b = d.getBranch(0);
            double railT = Double.NaN;
            List<Object> events = new ArrayList<>();
            for (FlightEvent e : b.getEvents()) {
                Map<String, Object> ev = new LinkedHashMap<>();
                ev.put("type", e.getType().toString());
                ev.put("time_s", e.getTime());
                events.add(ev);
                if (e.getType() == FlightEvent.Type.LAUNCHROD && Double.isNaN(railT)) railT = e.getTime();
            }
            r.put("events", events);
            List<Double> tt = b.get(FlightDataType.TYPE_TIME);
            List<Double> stab = b.get(FlightDataType.TYPE_STABILITY);
            List<Double> vz = b.get(FlightDataType.TYPE_VELOCITY_Z);
            List<Double> alt = b.get(FlightDataType.TYPE_ALTITUDE);
            double stabRail = Double.NaN;
            double descent = Double.NaN;
            if (tt != null && stab != null) {
                for (int i = 0; i < tt.size(); i++) {
                    if (!Double.isNaN(railT) && tt.get(i) >= railT && !Double.isNaN(stab.get(i))) {
                        stabRail = stab.get(i);
                        break;
                    }
                }
            }
            r.put("stability_off_rail_cal", stabRail);
            // Descent rate: mean downward speed over the last 5 s above the ground after apogee.
            if (tt != null && vz != null && alt != null && tt.size() > 2) {
                double apT = d.getTimeToApogee();
                double sum = 0;
                int n = 0;
                for (int i = 0; i < tt.size(); i++) {
                    if (tt.get(i) > apT && alt.get(i) > 5.0 && alt.get(i) < 60.0) {
                        sum += vz.get(i);
                        n++;
                    }
                }
                if (n > 0) descent = -sum / n;
            }
            r.put("descent_rate_ms", descent);
            r.put("warnings", sim.getSimulatedWarnings().toString());

            // Decimated time series of the primary branch.
            FlightDataType[] types = {FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE,
                    FlightDataType.TYPE_VELOCITY_Z, FlightDataType.TYPE_VELOCITY_TOTAL,
                    FlightDataType.TYPE_MACH_NUMBER, FlightDataType.TYPE_THRUST_FORCE,
                    FlightDataType.TYPE_DRAG_COEFF, FlightDataType.TYPE_CP_LOCATION,
                    FlightDataType.TYPE_CG_LOCATION, FlightDataType.TYPE_STABILITY, FlightDataType.TYPE_AOA,
                    FlightDataType.TYPE_MASS, FlightDataType.TYPE_ACCELERATION_TOTAL};
            String[] keys = {"t", "alt", "vz", "v", "mach", "thrust", "cd", "cp", "cg", "stab", "aoa", "mass",
                    "acc"};
            int len = b.getLength();
            int stride = Math.max(1, len / 400);
            List<Object> series = new ArrayList<>();
            for (int i = 0; i < len; i += stride) {
                Map<String, Object> row = new LinkedHashMap<>();
                for (int k = 0; k < types.length; k++) {
                    List<Double> col = b.get(types[k]);
                    row.put(keys[k], col == null ? null : col.get(i));
                }
                series.add(row);
            }
            r.put("series", series);
            m.put("results", r);
        } catch (Exception e) {
            m.put("error", e.toString());
        }
        o.setWindSpeedAverage(savedWind);
        o.setWindTurbulenceIntensity(savedTurb);
        o.setWindSpeedDeviation(savedDev);
        return m;
    }

    /** Minimal JSON writer (NaN/Inf -> null). */
    static final class Json {
        static String write(Object o) {
            StringBuilder sb = new StringBuilder();
            w(sb, o);
            sb.append('\n');
            return sb.toString();
        }

        @SuppressWarnings("unchecked")
        static void w(StringBuilder sb, Object o) {
            if (o == null) {
                sb.append("null");
            } else if (o instanceof Map) {
                sb.append('{');
                boolean first = true;
                for (Map.Entry<String, Object> e : ((Map<String, Object>) o).entrySet()) {
                    if (!first) sb.append(',');
                    first = false;
                    str(sb, e.getKey());
                    sb.append(':');
                    w(sb, e.getValue());
                }
                sb.append('}');
            } else if (o instanceof List) {
                sb.append('[');
                boolean first = true;
                for (Object e : (List<Object>) o) {
                    if (!first) sb.append(',');
                    first = false;
                    w(sb, e);
                }
                sb.append(']');
            } else if (o instanceof Double || o instanceof Float) {
                double d = ((Number) o).doubleValue();
                if (Double.isNaN(d) || Double.isInfinite(d)) sb.append("null");
                else sb.append(Double.toString(d));
            } else if (o instanceof Number || o instanceof Boolean) {
                sb.append(o);
            } else {
                str(sb, o.toString());
            }
        }

        static void str(StringBuilder sb, String s) {
            sb.append('"');
            for (char ch : s.toCharArray()) {
                switch (ch) {
                    case '"': sb.append("\\\""); break;
                    case '\\': sb.append("\\\\"); break;
                    case '\n': sb.append("\\n"); break;
                    case '\r': sb.append("\\r"); break;
                    case '\t': sb.append("\\t"); break;
                    default:
                        if (ch < 0x20) sb.append(String.format("\\u%04x", (int) ch));
                        else sb.append(ch);
                }
            }
            sb.append('"');
        }
    }
}

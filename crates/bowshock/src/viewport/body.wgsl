// The model shader. One uniform block decides the view mode (solid, cutaway, X-ray, exploded); vertices say which layer and
// system they belong to. Output is premultiplied alpha so the translucent shell of X-ray composites over the internals.

struct Uniforms {
    view_proj: mat4x4<f32>,
    light_dir: vec4<f32>,
    // Eye position; w is 1 for a perspective view (per-fragment view vector), 0 when orthographic (use view_dir).
    eye: vec4<f32>,
    view_dir: vec4<f32>,
    // mode (0 solid, 1 cutaway, 2 X-ray, 3 exploded), reveal (shell ghosted around a selected internal part), explode 0..1.
    params: vec4<f32>,
    // Cut wedge: centre direction xy, cos(half span), enabled.
    cut: vec4<f32>,
    // Physical pixels per metre at the target, device pixel ratio.
    screen: vec4<f32>,
    // body, fin, accent, structure, aero, propulsion, recovery, avionics, cut-face, cut-interior, bg-000 (linear).
    colors: array<vec4<f32>, 11>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) world: vec3<f32>,
    @location(2) edge: vec2<f32>,
    @location(3) state: f32,
    @location(4) material: f32,
    @location(5) system: f32,
    @location(6) layer: f32,
};

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) explode: vec3<f32>,
    @location(3) edge: vec2<f32>,
    @location(4) state: f32,
    @location(5) material: f32,
    @location(6) system: f32,
    @location(7) layer: f32,
) -> VsOut {
    var out: VsOut;
    let world = position + explode * u.params.z;
    out.clip = u.view_proj * vec4<f32>(world, 1.0);
    out.normal = normal;
    out.world = world;
    out.edge = edge;
    out.state = state;
    out.material = material;
    out.system = system;
    out.layer = layer;
    return out;
}

const BODY = 0;
const FIN = 1;
const ACCENT = 2;
const STRUCTURE = 3;
const CUT_FACE = 8;
const CUT_INTERIOR = 9;
const BG = 10;

@fragment
fn fs_main(in: VsOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let mode = i32(u.params.x + 0.5);
    let reveal = u.params.y > 0.5;
    let layer = i32(in.layer + 0.5);
    let sys = i32(in.system + 0.5);
    let ghosted = mode == 2 || (reveal && mode == 0);
    let accent = u.colors[ACCENT].rgb;

    // Cutaway: the wedge is removed from the airframe and the internals (fins and cord stay).
    if mode == 1 && u.cut.w > 0.5 && (layer == 0 || layer == 2) {
        let l = length(in.world.xy);
        if l > 1.0e-6 && dot(in.world.xy / l, u.cut.xy) > u.cut.z {
            discard;
        }
    }

    var n = normalize(in.normal);
    if !front {
        n = -n;
    }
    var view = u.view_dir.xyz;
    if u.eye.w > 0.5 {
        view = normalize(u.eye.xyz - in.world);
    }
    let facing = clamp(dot(n, view), 0.0, 1.0);
    let lit = 0.86 + 0.14 * max(dot(n, u.light_dir.xyz), 0.0);
    let rim = mix(0.45, 1.0, smoothstep(0.0, 0.55, facing));

    // Part states: 0 normal, 1 ghost (dragged in: pulled towards the accent), 2 hovered (a little brighter), 3 selected
    // (the silhouette edge tinted accent, like an outline), 4..5 flashing (just dropped or edited, strength state - 4), 6..7
    // flashing and selected (strength state - 6).
    var base = vec3<f32>(0.0);
    var alpha = 1.0;
    var shaded = true;

    if layer == 3 {
        // A cut wall: diagonal hatching in cut-face over the dark interior, in screen space.
        let spacing = 5.0 * u.screen.y;
        let p = (in.clip.x + in.clip.y) / spacing;
        let line = smoothstep(0.30, 0.38, abs(fract(p) - 0.5) * 2.0 - 0.0);
        base = mix(u.colors[CUT_INTERIOR].rgb, u.colors[CUT_FACE].rgb, 1.0 - line);
        shaded = false;
    } else if layer == 4 {
        // A cut solid: flat in its system colour.
        base = u.colors[STRUCTURE + sys].rgb;
        shaded = false;
    } else if layer == 0 {
        if !front && mode == 1 {
            // The inside of the shell seen through the cut.
            base = u.colors[CUT_INTERIOR].rgb * (0.6 + 0.4 * lit) + vec3<f32>(0.02) + u.colors[CUT_FACE].rgb * 0.5 * pow(1.0 - facing, 2.0);
            shaded = false;
        } else if ghosted {
            if !front {
                discard;
            }
            base = u.colors[BODY].rgb;
            alpha = 0.10 + 0.75 * pow(1.0 - facing, 2.0);
        } else {
            base = u.colors[BODY].rgb;
        }
    } else if layer == 1 {
        if mode == 1 || mode == 2 || mode == 3 {
            base = u.colors[STRUCTURE + 1].rgb;
            if mode == 2 {
                alpha = 0.22 + 0.6 * pow(1.0 - facing, 1.6);
            }
        } else {
            base = mix(u.colors[BODY].rgb, u.colors[FIN].rgb, in.material);
        }
    } else {
        // Internal solids (2) and cord (5): system colours; plain grey from outside in Solid (the motor's nozzle end).
        if mode == 0 && !reveal {
            base = u.colors[FIN].rgb;
        } else {
            base = u.colors[STRUCTURE + sys].rgb;
        }
    }

    if in.state > 2.5 {
        // Selected: the base colour is unchanged.
    } else if in.state > 1.5 {
        base = min(base * 1.1 + vec3<f32>(0.05), vec3<f32>(1.0));
    } else if in.state > 0.5 {
        base = mix(base, u.colors[ACCENT].rgb, 0.55);
    }

    var c = base;
    if shaded {
        c = base * lit * rim;
    }
    if in.state > 2.5 {
        // A flashing part (state 4 + strength, or 6 + strength when it is also selected) gets a wider, stronger outline and a
        // soft accent pulse over its body, both fading with the strength.
        var pulse = 0.0;
        var weight = 1.0;
        if in.state > 5.5 {
            pulse = in.state - 6.0;
        } else if in.state > 3.5 {
            pulse = in.state - 4.0;
            weight = pulse;
        }
        let k = pow(1.0 - facing, 3.0 - 0.8 * pulse);
        c = mix(c, accent, min(0.7 + 0.3 * pulse, 1.0) * k * weight);
        c = mix(c, accent, 0.08 * pulse);
        alpha = max(alpha, 0.8 * k * weight);
    }

    // A thin bg-000 line (50 %) where stacked parts meet.
    if layer == 0 && front && !ghosted {
        let e = min(in.edge.x, in.edge.y) * u.screen.x;
        let line = 1.0 - smoothstep(0.6, 1.4, e);
        c = mix(c, u.colors[BG].rgb, 0.5 * line);
    }
    // Stages that have dropped away in the phase on show (aft of `params.w`) are faint.
    if in.world.z > u.params.w {
        alpha = alpha * 0.18;
    }
    return vec4<f32>(c * alpha, alpha);
}

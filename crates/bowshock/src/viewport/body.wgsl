struct Uniforms {
    view_proj: mat4x4<f32>,
    light_dir: vec4<f32>,
    color: vec4<f32>,
    accent: vec4<f32>,
    fin: vec4<f32>,
    view_dir: vec4<f32>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) state: f32,
    @location(2) material: f32,
};

@vertex
fn vs_main(@location(0) position: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) state: f32, @location(3) material: f32) -> VsOut {
    var out: VsOut;
    out.clip = u.view_proj * vec4<f32>(position, 1.0);
    out.normal = normal;
    out.state = state;
    out.material = material;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let lit = 0.38 + 0.62 * max(dot(n, u.light_dir.xyz), 0.0);
    let facing = abs(dot(n, u.view_dir.xyz));
    let rim = mix(0.72, 1.0, smoothstep(0.0, 0.35, facing));
    // Part states: 0 normal, 1 ghost (dragged in: pulled towards the accent), 2 hovered (a little brighter), 3 selected
    // (the silhouette edge tinted accent, like an outline).
    let part = mix(u.color.rgb, u.fin.rgb, in.material);
    var base = part;
    if in.state > 2.5 {
        // Selected: the base colour is unchanged.
    } else if in.state > 1.5 {
        base = min(part * 1.1 + vec3<f32>(0.05), vec3<f32>(1.0));
    } else if in.state > 0.5 {
        base = mix(part, u.accent.rgb, 0.55);
    }
    var c = base * lit * rim;
    if in.state > 2.5 {
        c = mix(c, u.accent.rgb, 0.7 * pow(1.0 - facing, 3.0));
    }
    return vec4<f32>(c, 1.0);
}

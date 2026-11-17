// The flow picture: the cells of the grid as triangles with one scalar per vertex, coloured through a table, and optionally the
// cell edges as lines. The picture is drawn twice (instance 0 above the axis, instance 1 mirrored below it).

struct Uniforms {
    // centre of the view (x, r) in the units of the grid, then NDC per unit along x and along r (each includes the widget's size)
    xform: vec4<f32>,
    // colour range lo, hi; which of the three vertex values to colour by; unused
    range: vec4<f32>,
    // colour of the cell edges (premultiplied, linear)
    line: vec4<f32>,
    // the colour map, evenly spaced entries (linear RGB)
    lut: array<vec4<f32>, 64>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) value: f32,
};

fn place(pos: vec2<f32>, instance: u32) -> vec4<f32> {
    let r = select(pos.y, -pos.y, instance == 1u);
    return vec4<f32>((pos.x - u.xform.x) * u.xform.z, (r - u.xform.y) * u.xform.w, 0.0, 1.0);
}

@vertex
fn vs_fill(@builtin(instance_index) instance: u32, @location(0) pos: vec2<f32>, @location(1) values: vec3<f32>) -> VsOut {
    var out: VsOut;
    out.clip = place(pos, instance);
    let which = u32(u.range.z);
    out.value = select(select(values.x, values.y, which == 1u), values.z, which == 2u);
    return out;
}

@fragment
fn fs_fill(in: VsOut) -> @location(0) vec4<f32> {
    let t = clamp((in.value - u.range.x) / (u.range.y - u.range.x), 0.0, 1.0);
    let at = t * 63.0;
    let i = u32(floor(at));
    let j = min(i + 1u, 63u);
    let c = mix(u.lut[i], u.lut[j], at - floor(at));
    return vec4<f32>(c.rgb, 1.0);
}

@vertex
fn vs_line(@builtin(instance_index) instance: u32, @location(0) pos: vec2<f32>) -> @builtin(position) vec4<f32> {
    return place(pos, instance);
}

@fragment
fn fs_line() -> @location(0) vec4<f32> {
    return u.line;
}

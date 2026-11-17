// The 3D flow scene: the wall surface of the vehicle (lit, coloured by Cp or |Cf|) and the cut sheets (unlit, coloured by Mach, Cp, the
// schlieren or the eddy ratio). Vertices are in the solver's frame; a tile (a copy of the solved domain: a rotation or a reflection about
// the axis) is applied first, then the solver frame is permuted to the viewport's world frame: world = (y, z, x).

struct Tile {
    r0: vec4<f32>,
    r1: vec4<f32>,
    r2: vec4<f32>,
};

struct Uniforms {
    view_proj: mat4x4<f32>,
    // world position of the eye
    eye: vec4<f32>,
    // wall colour range lo, hi; the wall scalar's index; unused
    wall: vec4<f32>,
    // sheet colour range lo, hi; the sheet scalar's index; the number of tiles the sheets are drawn in
    sheet: vec4<f32>,
    // the two colour maps, evenly spaced entries (linear RGB)
    wall_lut: array<vec4<f32>, 64>,
    sheet_lut: array<vec4<f32>, 64>,
    tiles: array<Tile, 128>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

fn apply(t: Tile, p: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(dot(t.r0.xyz, p), dot(t.r1.xyz, p), dot(t.r2.xyz, p));
}

fn to_world(p: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(p.y, p.z, p.x);
}

fn lookup_wall(t: f32) -> vec3<f32> {
    let at = clamp(t, 0.0, 1.0) * 63.0;
    let i = u32(floor(at));
    let j = min(i + 1u, 63u);
    return mix(u.wall_lut[i].rgb, u.wall_lut[j].rgb, at - floor(at));
}

fn lookup_sheet(t: f32) -> vec3<f32> {
    let at = clamp(t, 0.0, 1.0) * 63.0;
    let i = u32(floor(at));
    let j = min(i + 1u, 63u);
    return mix(u.sheet_lut[i].rgb, u.sheet_lut[j].rgb, at - floor(at));
}

struct WallOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) value: f32,
};

@vertex
fn vs_wall(@builtin(instance_index) instance: u32, @location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) values: vec2<f32>) -> WallOut {
    let tile = u.tiles[instance];
    var out: WallOut;
    out.world = to_world(apply(tile, pos));
    out.normal = to_world(apply(tile, normal));
    out.clip = u.view_proj * vec4<f32>(out.world, 1.0);
    out.value = select(values.x, values.y, u32(u.wall.z) == 1u);
    return out;
}

@fragment
fn fs_wall(in: WallOut) -> @location(0) vec4<f32> {
    let t = (in.value - u.wall.x) / (u.wall.y - u.wall.x);
    let c = lookup_wall(t);
    // a headlight: the surface is lit by what faces the eye, either side of it (the fins are seen from both)
    let to_eye = normalize(u.eye.xyz - in.world);
    let lit = 0.38 + 0.62 * abs(dot(normalize(in.normal), to_eye));
    return vec4<f32>(c * lit, 1.0);
}

struct SheetOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) value: f32,
};

@vertex
fn vs_sheet(@builtin(instance_index) instance: u32, @location(0) pos: vec3<f32>, @location(1) values: vec4<f32>) -> SheetOut {
    let tile = u.tiles[instance];
    var out: SheetOut;
    out.clip = u.view_proj * vec4<f32>(to_world(apply(tile, pos)), 1.0);
    let which = u32(u.sheet.z);
    out.value = select(select(select(values.x, values.y, which == 1u), values.z, which == 2u), values.w, which == 3u);
    return out;
}

@fragment
fn fs_sheet(in: SheetOut) -> @location(0) vec4<f32> {
    let t = (in.value - u.sheet.x) / (u.sheet.y - u.sheet.x);
    return vec4<f32>(lookup_sheet(t), 1.0);
}

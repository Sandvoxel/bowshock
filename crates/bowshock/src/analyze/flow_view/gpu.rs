//! The GPU side of the flow picture: the cells of a [`Prepared`] field drawn as triangles into the widget's rectangle, coloured
//! through a table by one of three scalars per vertex, with the cell edges as lines on request. Pan, zoom and the choice of scalar
//! change a uniform block only; a new picture uploads its vertices (about 20 bytes per node and 24 per cell).
//!
//! The target already holds the screen: the pass loads it, draws inside the widget's rectangle (viewport and scissor) and leaves
//! the rest alone.

use super::map::{self, Scalar, LUT};
use super::prepared::{Prepared, Vertex};
use iced::wgpu::{self, util::DeviceExt};
use iced::widget::shader::{self, Viewport};
use iced::Rectangle;
use std::sync::Arc;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    /// Centre of the view `(x, r)`, then NDC per unit of length along `x` and along `r`.
    pub xform: [f32; 4],
    /// Colour range `lo`, `hi`; the scalar's index; unused.
    pub range: [f32; 4],
    /// Colour of the cell edges, premultiplied.
    pub line: [f32; 4],
    /// The colour map.
    pub lut: [[f32; 4]; LUT],
}

/// Which of the two instances of a picture (above the axis, and mirrored below it) is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Half {
    /// Both: the picture and its mirror image (an axisymmetric flow).
    Both,
    /// Above the axis only: the upper half of a picture whose lower half is another picture.
    Upper,
    /// Below the axis only: the picture, mirrored, is the lower half.
    Lower,
}

impl Half {
    /// The instances to draw.
    fn instances(self) -> std::ops::Range<u32> {
        match self {
            Half::Both => 0..2,
            Half::Upper => 0..1,
            Half::Lower => 1..2,
        }
    }
}

/// What one frame of the picture needs.
#[derive(Debug, Clone)]
pub struct Draw {
    pub picture: Arc<Prepared>,
    pub scalar: Scalar,
    pub dark: bool,
    pub grid: bool,
    /// The grid line colour (straight RGBA).
    pub line: iced::Color,
    /// Centre of the view in the grid's units, and logical pixels per unit.
    pub centre: (f64, f64),
    pub scale: f64,
    /// Which half of the plane this draw is.
    pub half: Half,
    /// The colour range of the scalar.
    pub range: (f32, f32),
}

impl shader::Program<super::Msg> for Draw {
    type State = ();
    type Primitive = Primitive;

    fn draw(&self, _: &(), _: iced::mouse::Cursor, bounds: Rectangle) -> Primitive {
        let (w, h) = (f64::from(bounds.width.max(1.0)), f64::from(bounds.height.max(1.0)));
        let (lo, hi) = self.range;
        let (line, a) = (iced::advanced::graphics::color::pack(iced::Color { a: 1.0, ..self.line }).components(), self.line.a);
        let uniforms = Uniforms {
            xform: [self.centre.0 as f32, self.centre.1 as f32, (2.0 * self.scale / w) as f32, (2.0 * self.scale / h) as f32],
            range: [lo, hi, self.scalar.index() as f32, 0.0],
            line: [line[0] * a, line[1] * a, line[2] * a, a],
            lut: map::lut(self.scalar.stops(self.dark)),
        };
        Primitive { picture: self.picture.clone(), uniforms, grid: self.grid, half: self.half }
    }
}

#[derive(Debug)]
pub struct Primitive {
    picture: Arc<Prepared>,
    uniforms: Uniforms,
    grid: bool,
    half: Half,
}

impl Primitive {
    /// What identifies this primitive's state in the pipeline: iced prepares every primitive of a frame before it renders any.
    fn key(&self) -> (u64, Half) {
        (self.picture.id, self.half)
    }
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(&self, pipeline: &mut Pipeline, device: &wgpu::Device, queue: &wgpu::Queue, bounds: &Rectangle, viewport: &Viewport) {
        pipeline.prepare(device, queue, &self.picture, &self.uniforms, self.grid, self.half, bounds, viewport);
    }

    fn render(&self, pipeline: &Pipeline, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView, clip_bounds: &Rectangle<u32>) {
        pipeline.render(self.key(), encoder, target, clip_bounds);
    }
}

/// The vertex and index buffers of a picture on the GPU.
struct Uploaded {
    id: u64,
    vertices: wgpu::Buffer,
    triangles: wgpu::Buffer,
    edges: wgpu::Buffer,
    triangle_indices: u32,
    edge_indices: u32,
}

/// One draw of a frame: the uniforms and what to draw, kept apart from the others because iced prepares every primitive of a frame
/// before it renders any (a picture drawn as two halves is two draws with different uniforms).
struct Item {
    key: (u64, Half),
    uniforms: wgpu::Buffer,
    group: wgpu::BindGroup,
    draw_grid: bool,
    /// Widget bounds in physical pixels: x, y, width, height.
    viewport: [f32; 4],
}

/// Pictures kept on the GPU and draws kept ready: the pictures of one frame, with a few to spare.
const KEEP: usize = 4;

pub struct Pipeline {
    fill: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    uploads: Vec<Uploaded>,
    items: Vec<Item>,
}

impl shader::Pipeline for Pipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("flow.layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("flow.shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("flow.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("flow.pipeline_layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let premultiplied = wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::One,
                dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::One,
                dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                operation: wgpu::BlendOperation::Add,
            },
        };
        let make = |label: &str, vs: &str, fs: &str, topology: wgpu::PrimitiveTopology, blend: wgpu::BlendState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some(vs),
                    buffers: &[Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Vertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x2, 1 => Float32x3],
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                primitive: wgpu::PrimitiveState { topology, front_face: wgpu::FrontFace::Ccw, cull_mode: None, ..Default::default() },
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    targets: &[Some(wgpu::ColorTargetState { format, blend: Some(blend), write_mask: wgpu::ColorWrites::ALL })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let fill = make("flow.fill", "vs_fill", "fs_fill", wgpu::PrimitiveTopology::TriangleList, wgpu::BlendState::REPLACE);
        let lines = make("flow.lines", "vs_line", "fs_line", wgpu::PrimitiveTopology::LineList, premultiplied);
        Pipeline { fill, lines, layout, uploads: Vec::new(), items: Vec::new() }
    }
}

impl Pipeline {
    #[allow(clippy::too_many_arguments)]
    fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        picture: &Prepared,
        uniforms: &Uniforms,
        grid: bool,
        half: Half,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        if !self.uploads.iter().any(|u| u.id == picture.id) && !picture.triangles.is_empty() {
            let buffer =
                |label: &str, contents: &[u8], usage| device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some(label), contents, usage });
            if self.uploads.len() >= KEEP {
                self.uploads.remove(0);
            }
            self.uploads.push(Uploaded {
                id: picture.id,
                vertices: buffer("flow.vertices", bytemuck::cast_slice(&picture.vertices), wgpu::BufferUsages::VERTEX),
                triangles: buffer("flow.triangles", bytemuck::cast_slice(&picture.triangles), wgpu::BufferUsages::INDEX),
                // wgpu rejects empty buffers: a grid without edges draws one degenerate segment
                edges: buffer(
                    "flow.edges",
                    bytemuck::cast_slice(if picture.edges.is_empty() { &[0u32, 0] } else { &picture.edges }),
                    wgpu::BufferUsages::INDEX,
                ),
                triangle_indices: picture.triangles.len() as u32,
                edge_indices: picture.edges.len() as u32,
            });
        }
        let key = (picture.id, half);
        let at = match self.items.iter().position(|i| i.key == key) {
            Some(at) => at,
            None => {
                if self.items.len() >= 2 * KEEP {
                    self.items.remove(0);
                }
                let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("flow.uniforms"),
                    size: std::mem::size_of::<Uniforms>() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("flow.group"),
                    layout: &self.layout,
                    entries: &[wgpu::BindGroupEntry { binding: 0, resource: buffer.as_entire_binding() }],
                });
                self.items.push(Item { key, uniforms: buffer, group, draw_grid: false, viewport: [0.0; 4] });
                self.items.len() - 1
            }
        };
        let item = &mut self.items[at];
        queue.write_buffer(&item.uniforms, 0, bytemuck::bytes_of(uniforms));
        item.draw_grid = grid;
        let scale = viewport.scale_factor();
        item.viewport = [bounds.x * scale, bounds.y * scale, bounds.width * scale, bounds.height * scale];
    }

    fn render(&self, key: (u64, Half), encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView, clip: &Rectangle<u32>) {
        let (Some(up), Some(item)) = (self.uploads.iter().find(|u| u.id == key.0), self.items.iter().find(|i| i.key == key)) else { return };
        if clip.width == 0 || clip.height == 0 || item.viewport[2] < 1.0 || item.viewport[3] < 1.0 {
            return;
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("flow.pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        let [x, y, w, h] = item.viewport;
        pass.set_viewport(x, y, w, h, 0.0, 1.0);
        pass.set_scissor_rect(clip.x, clip.y, clip.width, clip.height);
        pass.set_bind_group(0, &item.group, &[]);
        pass.set_vertex_buffer(0, up.vertices.slice(..));
        pass.set_pipeline(&self.fill);
        pass.set_index_buffer(up.triangles.slice(..), wgpu::IndexFormat::Uint32);
        let instances = key.1.instances();
        pass.draw_indexed(0..up.triangle_indices, 0, instances.clone());
        if item.draw_grid && up.edge_indices > 0 {
            pass.set_pipeline(&self.lines);
            pass.set_index_buffer(up.edges.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..up.edge_indices, 0, instances);
        }
    }
}
#[cfg(test)]
mod tests {
    use super::super::prepared::tests::sized;
    use super::*;
    use iced::futures::executor::block_on;
    use iced::widget::shader::{Pipeline as _, Primitive as _, Program as _};
    use iced::{mouse, Point, Size};
    use std::time::Instant;

    /// The real render on a real device (skipped where there is no adapter): a uniform field at its top Mach number is the top colour
    /// of the map, the picture is mirrored about the axis, outside the grid the target is left alone, and a frame of a coarse-sized grid
    /// costs milliseconds. `BOWSHOCK_FLOW_FRAMES=<n>` sets how many frames the cost is averaged over (default 60).
    #[test]
    fn the_picture_renders_on_a_device_in_the_colours_of_its_map_and_costs_milliseconds_a_frame() {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let Ok(adapter) = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())) else {
            eprintln!("no graphics adapter: flow render test skipped");
            return;
        };
        let (device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).expect("a device");
        // the app's colours are packed as written (iced's `web-colors`), for a target that is not sRGB
        let format = wgpu::TextureFormat::Rgba8Unorm;
        let mut pipeline = Pipeline::new(&device, &queue, format);
        let (w, h) = (1408u32, 500u32);
        // Mach 1.5 everywhere: the colour range is 0 to 1.5, so every cell is the top of the map
        let mut field = sized(220, 124);
        field.prim.iter_mut().for_each(|q| *q = [1.0, 1.5, 0.0, 0.0, (1.0 / bowshock_core::field::GAMMA) as f32]);
        let picture = Arc::new(Prepared::new(Arc::new(field)));
        assert_eq!(picture.ranges[Scalar::Mach.index()], (0.0, 1.5));
        let draw = Draw {
            picture,
            scalar: Scalar::Mach,
            dark: true,
            grid: false,
            line: iced::Color::WHITE,
            centre: (0.7, 0.0),
            scale: 600.0,
            half: Half::Both,
            range: (0.0, 1.5),
        };
        let bounds = Rectangle::new(Point::ORIGIN, Size::new(w as f32, h as f32));
        let viewport = Viewport::with_physical_size(Size::new(w, h), iced::advanced::renderer::Scale::default());
        let target = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("flow.test.target"),
            size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = target.create_view(&wgpu::TextureViewDescriptor::default());
        let background = wgpu::Color::BLACK;
        let frame = |pipeline: &mut Pipeline, draw: &Draw, readback: Option<&wgpu::Buffer>| {
            let primitive = draw.draw(&(), mouse::Cursor::Unavailable, bounds);
            primitive.prepare(pipeline, &device, &queue, &bounds, &viewport);
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("flow.test") });
            drop(encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("flow.test.clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(background), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            }));
            primitive.render(pipeline, &mut encoder, &view, &Rectangle { x: 0, y: 0, width: w, height: h });
            if let Some(buffer) = readback {
                encoder.copy_texture_to_buffer(
                    wgpu::TexelCopyTextureInfo { texture: &target, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                    wgpu::TexelCopyBufferInfo {
                        buffer,
                        layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(w * 4), rows_per_image: Some(h) },
                    },
                    wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
                );
            }
            queue.submit(Some(encoder.finish()));
        };

        // one frame, read back (1408 x 4 bytes is a multiple of the 256-byte row alignment)
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("flow.test.readback"),
            size: u64::from(w * h * 4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        frame(&mut pipeline, &draw, Some(&readback));
        let (tx, rx) = std::sync::mpsc::channel();
        readback.slice(..).map_async(wgpu::MapMode::Read, move |r| drop(tx.send(r)));
        device.poll(wgpu::PollType::wait_indefinitely()).expect("the device finished");
        rx.recv().unwrap().expect("mapped");
        let pixels = readback.slice(..).get_mapped_range().expect("range").to_vec();
        let at = |x: u32, y: u32| -> [u8; 4] {
            let i = ((y * w + x) * 4) as usize;
            [pixels[i], pixels[i + 1], pixels[i + 2], pixels[i + 3]]
        };
        let near = |a: [u8; 4], b: [u8; 3]| a[..3].iter().zip(b).all(|(x, y)| (i32::from(*x) - i32::from(y)).abs() <= 3);
        // the grid spans x in [-0.2, 1.6] and the view -0.47 to 1.87: its left edge is outside, the middle is the top of viridis
        assert!(near(at(5, 250), [0, 0, 0]), "outside the grid the target is as it was: {:?}", at(5, 250));
        assert!(near(at(700, 250), map::VIRIDIS[8]), "the top of the map: {:?}", at(700, 250));
        // the picture is mirrored about the axis (the pixel row of r = 0 is 250)
        assert_eq!(at(700, 150), at(700, 350));
        assert!(near(at(700, 150), map::VIRIDIS[8]));
        // the view spans r = ±0.42 and the grid ±0.3: above and below it the target is as it was
        assert!(near(at(700, 5), [0, 0, 0]) && near(at(700, 495), [0, 0, 0]));

        // the cost of a frame: the upload happened once; each frame is a uniform write and a draw of every cell twice (the mirror).
        // The coarse grid of the app (27 000 cells here), and one four times as large; with and without the cell edges.
        let frames: usize = std::env::var("BOWSHOCK_FLOW_FRAMES").ok().and_then(|s| s.parse().ok()).unwrap_or(60);
        let big = Prepared::new(Arc::new(sized(440, 248)));
        for (cells, picture) in [(220 * 124, draw.picture.clone()), (440 * 248, Arc::new(big))] {
            for grid in [false, true] {
                let case = Draw { picture: picture.clone(), grid, ..draw.clone() };
                frame(&mut pipeline, &case, None);
                device.poll(wgpu::PollType::wait_indefinitely()).expect("the device finished");
                let t = Instant::now();
                for _ in 0..frames {
                    frame(&mut pipeline, &case, None);
                    device.poll(wgpu::PollType::wait_indefinitely()).expect("the device finished");
                }
                let per_frame = t.elapsed().as_secs_f64() * 1e3 / frames as f64;
                eprintln!(
                    "flow picture, {cells} cells{}, {w} x {h} px: {per_frame:.2} ms a frame (submit to done), adapter {}",
                    if grid { " and their edges" } else { "" },
                    adapter.get_info().name
                );
                assert!(per_frame < 250.0, "{per_frame} ms a frame");
            }
        }
    }
}

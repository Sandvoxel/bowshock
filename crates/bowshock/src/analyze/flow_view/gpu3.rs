//! The GPU side of the 3D flow scene: the wall surface of a [`Prepared3`] and its cut sheets drawn with depth into a 4x MSAA offscreen
//! texture, then composited over the widget's rectangle (the target already holds the screen and is never cleared), like the viewport.
//!
//! What goes up: the wall mesh once per picture (about 32 bytes a vertex, a few 10^5 vertices at most) and each cut sheet when it is
//! built (28 bytes a vertex, a few 10^4 vertices). The cells never go up. Camera, scalar, colour ranges and the symmetry tiles are one
//! uniform block, so orbiting, changing what the colours show and toggling the tiling cost a uniform write. A tile is an instance:
//! the whole rocket is the same buffers drawn once per copy of the solved domain.

use super::map::{self, LUT};
use super::scene3::{Prepared3, Scalar3, SheetDraw, TileSel, WallScalar, WallVertex, Window};
use crate::viewport::camera::Camera;
use iced::wgpu::{self, util::DeviceExt};
use iced::widget::shader::{self, Viewport};
use iced::{Rectangle, Size};
use std::sync::Arc;

const SAMPLES: u32 = 4;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// Most tiles one draw takes (the uniform block holds this many).
pub const MAX_TILES: usize = 128;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    pub view_proj: [[f32; 4]; 4],
    /// World position of the eye.
    pub eye: [f32; 4],
    /// Wall colour range `lo`, `hi`; the wall scalar's index; unused.
    pub wall: [f32; 4],
    /// Sheet colour range `lo`, `hi`; the sheet scalar's index; unused.
    pub sheet: [f32; 4],
    /// The window the sheets are clipped to: axial range `lo`, `hi`, largest radius, unused.
    pub window: [f32; 4],
    pub wall_lut: [[f32; 4]; LUT],
    pub sheet_lut: [[f32; 4]; LUT],
    /// The rows of each tile's matrix.
    pub tiles: [[[f32; 4]; 3]; MAX_TILES],
}

/// What one frame of the scene shows.
#[derive(Debug, Clone)]
pub struct View3 {
    pub camera: Camera,
    pub scalar: Scalar3,
    pub wall_scalar: WallScalar,
    /// The cut sheets drawn, already built, each with the tiles it is drawn in.
    pub sheets: Vec<SheetDraw>,
    /// What the sheets are clipped to.
    pub window: Window,
    /// The colour range of the sheet scalar and of the wall scalar in use.
    pub sheet_range: (f32, f32),
    pub wall_range: (f32, f32),
    /// Draw the wall surface in every tile, not only in the solved domain.
    pub tile_walls: bool,
    /// Draw the wall surface.
    pub walls: bool,
    pub dark: bool,
}

/// The uniform block of `view` of `picture` in a viewport of `size` logical pixels.
pub fn uniforms(picture: &Prepared3, view: &View3, size: Size) -> Uniforms {
    let (wlo, whi) = view.wall_range;
    let (slo, shi) = view.sheet_range;
    let eye = view.camera.eye(size);
    let mut tiles = [[[0.0; 4]; 3]; MAX_TILES];
    for (slot, t) in tiles.iter_mut().zip(&picture.tiles) {
        *slot = t.m.map(|row| [row[0], row[1], row[2], 0.0]);
    }
    Uniforms {
        view_proj: view.camera.view_proj(size).to_cols_array_2d(),
        eye: [eye.x, eye.y, eye.z, 1.0],
        wall: [wlo, whi, view.wall_scalar.index() as f32, 0.0],
        sheet: [slo, shi, view.scalar.index() as f32, 0.0],
        window: [view.window.x.0, view.window.x.1, view.window.r_max, 0.0],
        wall_lut: map::lut(view.wall_scalar.stops()),
        sheet_lut: map::lut(view.scalar.stops(view.dark)),
        tiles,
    }
}

/// What one frame of the scene needs: the picture and the view.
#[derive(Debug, Clone)]
pub struct Draw3 {
    pub picture: Arc<Prepared3>,
    pub view: View3,
}

impl shader::Program<super::Msg> for Draw3 {
    type State = ();
    type Primitive = Primitive;

    fn draw(&self, _: &(), _: iced::mouse::Cursor, bounds: Rectangle) -> Primitive {
        Primitive { picture: self.picture.clone(), view: self.view.clone(), uniforms: uniforms(&self.picture, &self.view, bounds.size()) }
    }
}

#[derive(Debug)]
pub struct Primitive {
    picture: Arc<Prepared3>,
    view: View3,
    uniforms: Uniforms,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(&self, pipeline: &mut Pipeline, device: &wgpu::Device, queue: &wgpu::Queue, bounds: &Rectangle, viewport: &Viewport) {
        pipeline.prepare(device, queue, &self.picture, &self.view, &self.uniforms, bounds, viewport);
    }

    fn render(&self, pipeline: &Pipeline, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView, clip_bounds: &Rectangle<u32>) {
        pipeline.render(encoder, target, clip_bounds);
    }
}

struct WallGpu {
    id: u64,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    body_indices: u32,
}

struct SheetGpu {
    id: u64,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
}

struct Offscreen {
    size: (u32, u32),
    msaa_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    blit_group: wgpu::BindGroup,
}

pub struct Pipeline {
    format: wgpu::TextureFormat,
    wall: wgpu::RenderPipeline,
    sheet: wgpu::RenderPipeline,
    blit: wgpu::RenderPipeline,
    uniforms: wgpu::Buffer,
    group: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    walls: Option<WallGpu>,
    /// The sheets on the GPU, by mesh id.
    sheets: Vec<SheetGpu>,
    draw_walls: bool,
    tiles: u32,
    /// What to draw: an index into `sheets` and the instances (tiles) to draw it in.
    sheet_draws: Vec<(usize, std::ops::Range<u32>)>,
    offscreen: Option<Offscreen>,
    /// Widget bounds in physical pixels: x, y, width, height.
    viewport: [f32; 4],
}

impl shader::Pipeline for Pipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("flow3.uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("flow3.layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("flow3.group"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: uniforms.as_entire_binding() }],
        });
        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("flow3.blit_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            }],
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("flow3.shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("scene3.wgsl").into()),
        });
        let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("flow3.blit"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../viewport/blit.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("flow3.pipeline_layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let blit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("flow3.blit_pipeline_layout"),
            bind_group_layouts: &[Some(&blit_layout)],
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
        let make = |label: &str, vs: &str, fs: &str, stride: u64, attributes: &[wgpu::VertexAttribute], bias: wgpu::DepthBiasState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some(vs),
                    buffers: &[Some(wgpu::VertexBufferLayout { array_stride: stride, step_mode: wgpu::VertexStepMode::Vertex, attributes })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: None,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::Less),
                    stencil: wgpu::StencilState::default(),
                    bias,
                }),
                multisample: wgpu::MultisampleState { count: SAMPLES, mask: !0, alpha_to_coverage_enabled: false },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    targets: &[Some(wgpu::ColorTargetState { format, blend: Some(wgpu::BlendState::REPLACE), write_mask: wgpu::ColorWrites::ALL })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let wall = make(
            "flow3.wall",
            "vs_wall",
            "fs_wall",
            std::mem::size_of::<WallVertex>() as u64,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2],
            wgpu::DepthBiasState::default(),
        );
        // a sheet that touches the wall (the cut layer's edge lies on the wall faces) wins the depth test
        let sheet = make(
            "flow3.sheet",
            "vs_sheet",
            "fs_sheet",
            std::mem::size_of::<super::scene3::SheetVertex>() as u64,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
            wgpu::DepthBiasState { constant: -2, slope_scale: -1.0, clamp: 0.0 },
        );
        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("flow3.blit_pipeline"),
            layout: Some(&blit_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &blit_shader,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &blit_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState { format, blend: Some(premultiplied), write_mask: wgpu::ColorWrites::ALL })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });
        Pipeline {
            format,
            wall,
            sheet,
            blit,
            uniforms,
            group,
            blit_layout,
            walls: None,
            sheets: Vec::new(),
            draw_walls: true,
            tiles: 1,
            sheet_draws: Vec::new(),
            offscreen: None,
            viewport: [0.0; 4],
        }
    }
}

impl Pipeline {
    #[allow(clippy::too_many_arguments)]
    fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        picture: &Prepared3,
        view: &View3,
        uniforms: &Uniforms,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        let physical = viewport.physical_size();
        let size = (physical.width.max(1), physical.height.max(1));
        if self.offscreen.as_ref().is_none_or(|o| o.size != size) {
            self.offscreen = Some(self.create_offscreen(device, size));
        }
        let buffer = |label: &str, contents: &[u8], usage| device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some(label), contents, usage });
        if self.walls.as_ref().is_none_or(|w| w.id != picture.id) {
            let m = &picture.walls;
            self.walls = (m.body_indices > 0).then(|| WallGpu {
                id: picture.id,
                vertices: buffer("flow3.wall_vertices", bytemuck::cast_slice(&m.vertices), wgpu::BufferUsages::VERTEX),
                indices: buffer("flow3.wall_indices", bytemuck::cast_slice(&m.triangles[..m.body_indices as usize]), wgpu::BufferUsages::INDEX),
                body_indices: m.body_indices,
            });
        }
        // the sheets drawn: upload those not on the GPU yet, drop those no longer drawn
        self.sheets.retain(|g| view.sheets.iter().any(|s| s.mesh.id == g.id));
        for d in &view.sheets {
            let s = &d.mesh;
            if s.triangles.is_empty() || self.sheets.iter().any(|g| g.id == s.id) {
                continue;
            }
            self.sheets.push(SheetGpu {
                id: s.id,
                vertices: buffer("flow3.sheet_vertices", bytemuck::cast_slice(&s.vertices), wgpu::BufferUsages::VERTEX),
                indices: buffer("flow3.sheet_indices", bytemuck::cast_slice(&s.triangles), wgpu::BufferUsages::INDEX),
                count: s.triangles.len() as u32,
            });
        }
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(uniforms));
        self.draw_walls = view.walls;
        let tiles = picture.tiles.len().clamp(1, MAX_TILES) as u32;
        self.tiles = if view.tile_walls { tiles } else { 1 };
        self.sheet_draws = view
            .sheets
            .iter()
            .filter_map(|d| {
                let at = self.sheets.iter().position(|g| g.id == d.mesh.id)?;
                let instances = match d.tiles {
                    TileSel::All => 0..tiles,
                    TileSel::One(t) => t.min(tiles - 1)..t.min(tiles - 1) + 1,
                };
                Some((at, instances))
            })
            .collect();
        let scale = viewport.scale_factor();
        self.viewport = [bounds.x * scale, bounds.y * scale, bounds.width * scale, bounds.height * scale];
    }

    fn create_offscreen(&self, device: &wgpu::Device, size: (u32, u32)) -> Offscreen {
        let extent = wgpu::Extent3d { width: size.0, height: size.1, depth_or_array_layers: 1 };
        let texture = |label: &str, format, samples, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: extent,
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let view = |texture: &wgpu::Texture| texture.create_view(&wgpu::TextureViewDescriptor::default());
        let msaa_view = view(&texture("flow3.msaa", self.format, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT));
        let resolve_view = view(&texture("flow3.resolve", self.format, 1, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING));
        let depth_view = view(&texture("flow3.depth", DEPTH_FORMAT, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT));
        let blit_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("flow3.blit_group"),
            layout: &self.blit_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&resolve_view) }],
        });
        Offscreen { size, msaa_view, resolve_view, depth_view, blit_group }
    }

    fn render(&self, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView, clip: &Rectangle<u32>) {
        let Some(off) = &self.offscreen else { return };
        if clip.width == 0 || clip.height == 0 || self.viewport[2] < 1.0 || self.viewport[3] < 1.0 {
            return;
        }
        // keep the scissor inside the offscreen texture
        let sx = clip.x.min(off.size.0.saturating_sub(1));
        let sy = clip.y.min(off.size.1.saturating_sub(1));
        let sw = clip.width.min(off.size.0 - sx).max(1);
        let sh = clip.height.min(off.size.1 - sy).max(1);
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("flow3.pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &off.msaa_view,
                    depth_slice: None,
                    resolve_target: Some(&off.resolve_view),
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT), store: wgpu::StoreOp::Discard },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &off.depth_view,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Discard }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            let [x, y, w, h] = self.viewport;
            pass.set_viewport(x, y, w.max(1.0), h.max(1.0), 0.0, 1.0);
            pass.set_scissor_rect(sx, sy, sw, sh);
            pass.set_bind_group(0, &self.group, &[]);
            if let (true, Some(walls)) = (self.draw_walls, &self.walls) {
                pass.set_pipeline(&self.wall);
                pass.set_vertex_buffer(0, walls.vertices.slice(..));
                pass.set_index_buffer(walls.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..walls.body_indices, 0, 0..self.tiles);
            }
            pass.set_pipeline(&self.sheet);
            for (at, instances) in &self.sheet_draws {
                let s = &self.sheets[*at];
                pass.set_vertex_buffer(0, s.vertices.slice(..));
                pass.set_index_buffer(s.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..s.count, 0, instances.clone());
            }
        }
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("flow3.blit_pass"),
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
            pass.set_viewport(0.0, 0.0, off.size.0 as f32, off.size.1 as f32, 0.0, 1.0);
            pass.set_scissor_rect(sx, sy, sw, sh);
            pass.set_pipeline(&self.blit);
            pass.set_bind_group(0, &off.blit_group, &[]);
            pass.draw(0..3, 0..1);
        }
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::super::scene3::tests::anf;
    use super::super::scene3::Sheet;
    use super::*;
    use iced::futures::executor::block_on;
    use iced::widget::shader::{Pipeline as _, Primitive as _, Program as _};
    use iced::{mouse, Point};
    use std::time::Instant;

    /// A device for the tests, or `None` where there is no adapter.
    pub(in crate::analyze) fn device() -> Option<(wgpu::Adapter, wgpu::Device, wgpu::Queue)> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok()?;
        let (device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).ok()?;
        Some((adapter, device, queue))
    }

    /// The target a test renders into, and what reads it back.
    pub(in crate::analyze) struct Rig {
        pub device: wgpu::Device,
        pub queue: wgpu::Queue,
        pub pipeline: Pipeline,
        target: wgpu::Texture,
        view: wgpu::TextureView,
        readback: wgpu::Buffer,
        pub size: (u32, u32),
        pub background: wgpu::Color,
    }

    impl Rig {
        pub fn new(device: wgpu::Device, queue: wgpu::Queue, size: (u32, u32), background: wgpu::Color) -> Rig {
            // the app's colours are packed as written (iced's `web-colors`), for a target that is not sRGB
            let format = wgpu::TextureFormat::Rgba8Unorm;
            let pipeline = Pipeline::new(&device, &queue, format);
            let target = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("flow3.test.target"),
                size: wgpu::Extent3d { width: size.0, height: size.1, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            });
            let view = target.create_view(&wgpu::TextureViewDescriptor::default());
            assert_eq!((size.0 * 4) % 256, 0, "the readback rows must be a multiple of 256 bytes");
            let readback = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("flow3.test.readback"),
                size: u64::from(size.0 * size.1 * 4),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            Rig { device, queue, pipeline, target, view, readback, size, background }
        }

        /// One frame of `draw`, optionally read back (RGBA8, top row first).
        pub fn frame(&mut self, draw: &Draw3, read: bool) -> Option<Vec<u8>> {
            let (w, h) = self.size;
            let bounds = Rectangle::new(Point::ORIGIN, Size::new(w as f32, h as f32));
            let viewport = Viewport::with_physical_size(Size::new(w, h), iced::advanced::renderer::Scale::default());
            let primitive = draw.draw(&(), mouse::Cursor::Unavailable, bounds);
            primitive.prepare(&mut self.pipeline, &self.device, &self.queue, &bounds, &viewport);
            let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("flow3.test") });
            drop(encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("flow3.test.clear"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(self.background), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            }));
            primitive.render(&self.pipeline, &mut encoder, &self.view, &Rectangle { x: 0, y: 0, width: w, height: h });
            if read {
                encoder.copy_texture_to_buffer(
                    wgpu::TexelCopyTextureInfo { texture: &self.target, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                    wgpu::TexelCopyBufferInfo {
                        buffer: &self.readback,
                        layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(w * 4), rows_per_image: Some(h) },
                    },
                    wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
                );
            }
            self.queue.submit(Some(encoder.finish()));
            if !read {
                return None;
            }
            let (tx, rx) = std::sync::mpsc::channel();
            self.readback.slice(..).map_async(wgpu::MapMode::Read, move |r| drop(tx.send(r)));
            self.device.poll(wgpu::PollType::wait_indefinitely()).expect("the device finished");
            rx.recv().unwrap().expect("mapped");
            let pixels = self.readback.slice(..).get_mapped_range().expect("range").to_vec();
            self.readback.unmap();
            Some(pixels)
        }

        pub fn wait(&self) {
            self.device.poll(wgpu::PollType::wait_indefinitely()).expect("the device finished");
        }
    }

    /// Writes RGBA8 `pixels` as a PNG.
    pub(in crate::analyze) fn save_png(path: &std::path::Path, pixels: &[u8], size: (u32, u32)) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let mut enc = png::Encoder::new(std::io::BufWriter::new(std::fs::File::create(path).unwrap()), size.0, size.1);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header().unwrap().write_image_data(pixels).unwrap();
    }

    /// Looking at the solved half domain from its open side (`+X`, ahead of the nose): the cut sheet is behind the half body, nose to the right.
    pub(in crate::analyze) const FROM_PLUS_X: f32 = std::f32::consts::FRAC_PI_2 + 0.7;
    /// Looking at the whole rocket from the `-X` side, ahead of the nose, nose to the left.
    pub(in crate::analyze) const FROM_MINUS_X: f32 = -std::f32::consts::FRAC_PI_2 - 0.7;

    /// A three-quarter view of the rocket from above the lee side, from `yaw`. With `sheet` the box that is framed also holds the lee
    /// half of the cut sheet's window.
    pub(in crate::analyze) fn camera_for(picture: &Prepared3, size: (u32, u32), sheet: bool, yaw: f32) -> Camera {
        let mut camera = Camera { yaw, pitch: 0.38, ..Camera::default() };
        let (lo, mut hi) = picture.bounds;
        if sheet {
            hi[1] = hi[1].max(0.6 * picture.window.r_max);
        }
        camera.fit(glam::Vec3::from(lo), glam::Vec3::from(hi), Size::new(size.0 as f32, size.1 as f32));
        camera
    }

    fn lit(pixels: &[u8], bg: [u8; 3]) -> usize {
        pixels.chunks(4).filter(|p| p[..3].iter().zip(bg).any(|(a, b)| (i32::from(*a) - i32::from(b)).abs() > 6)).count()
    }

    /// The view the tests start from: walls in every tile, the picture's own colour ranges, nothing cut.
    pub(in crate::analyze) fn view_of(picture: &Prepared3, camera: Camera) -> View3 {
        View3 {
            camera,
            scalar: Scalar3::Mach,
            wall_scalar: WallScalar::Cp,
            sheets: Vec::new(),
            window: picture.window,
            sheet_range: picture.ranges[Scalar3::Mach.index()],
            wall_range: picture.wall_ranges[WallScalar::Cp.index()],
            tile_walls: true,
            walls: true,
            dark: true,
        }
    }

    fn rig_for(size: (u32, u32)) -> Option<(wgpu::Adapter, Rig, [u8; 3])> {
        let (adapter, device, queue) = device()?;
        let bg = [0x0C, 0x0E, 0x11];
        let color = wgpu::Color { r: f64::from(bg[0]) / 255.0, g: f64::from(bg[1]) / 255.0, b: f64::from(bg[2]) / 255.0, a: 1.0 };
        Some((adapter, Rig::new(device, queue, size, color), bg))
    }

    /// The real render on a real device (skipped where there is no adapter) of the ANF fixture: walls coloured by Cp and a meridian Mach
    /// sheet, the same tiled to the whole rocket, and the cost of a frame. With `BOWSHOCK_FLOW_OUT=<dir>` the pictures are written there.
    #[test]
    fn the_finner_renders_with_its_walls_and_sheets_and_costs_milliseconds_a_frame() {
        let size = (1280u32, 720u32);
        let Some((adapter, mut rig, bg)) = rig_for(size) else {
            eprintln!("no graphics adapter: flow3 render test skipped");
            return;
        };
        let picture = Arc::new(Prepared3::new(anf()));
        let k = picture.field.layer_of_theta(0.0);
        let plane = picture.sheet_draws(Sheet::Meridian(k), picture.window, false);
        let out = std::env::var_os("BOWSHOCK_FLOW_OUT").map(std::path::PathBuf::from);
        let shot = |name: &str, pixels: &[u8]| {
            if let Some(dir) = &out {
                save_png(&dir.join(name), pixels, size);
            }
        };

        // walls only, the solved half domain: the background is left alone far from the rocket and the rocket is drawn
        let walls_only =
            Draw3 { picture: picture.clone(), view: View3 { tile_walls: false, ..view_of(&picture, camera_for(&picture, size, false, FROM_PLUS_X)) } };
        let px = rig.frame(&walls_only, true).unwrap();
        assert_eq!(&px[..3], &bg, "the corner is the background");
        let wall_pixels = lit(&px, bg);
        assert!(wall_pixels > 5_000, "the rocket is drawn: {wall_pixels} px");
        shot("scene-walls-cp.png", &px);

        // the meridian plane with the half domain's walls, seen from its open side
        let solved = Draw3 {
            picture: picture.clone(),
            view: View3 { sheets: plane.clone(), tile_walls: false, ..view_of(&picture, camera_for(&picture, size, true, FROM_PLUS_X)) },
        };
        let px = rig.frame(&solved, true).unwrap();
        assert!(lit(&px, bg) > wall_pixels + 5_000, "the sheet adds area: {} vs {wall_pixels}", lit(&px, bg));
        shot("scene-walls-cp-meridian-mach.png", &px);

        // the whole rocket in its tiles, the whole plane
        let tiled =
            Draw3 { picture: picture.clone(), view: View3 { sheets: plane.clone(), ..view_of(&picture, camera_for(&picture, size, true, FROM_MINUS_X)) } };
        let px = rig.frame(&tiled, true).unwrap();
        shot("scene-walls-cp-meridian-mach-tiled.png", &px);

        // a cross-flow disc and the schlieren, from the side
        let cross = View3 {
            camera: Camera { yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.2, ..camera_for(&picture, size, true, FROM_MINUS_X) },
            sheets: [plane.clone(), picture.sheet_draws(Sheet::Cross(picture.field.ni * 3 / 5), picture.window, false)].concat(),
            scalar: Scalar3::Schlieren,
            sheet_range: picture.ranges[Scalar3::Schlieren.index()],
            ..view_of(&picture, camera_for(&picture, size, true, FROM_MINUS_X))
        };
        let px = rig.frame(&Draw3 { picture: picture.clone(), view: cross }, true).unwrap();
        shot("scene-schlieren-cross-tiled.png", &px);

        // the cost of a frame: the uploads happened once; each frame is a uniform write and the draws
        let frames: usize = std::env::var("BOWSHOCK_FLOW_FRAMES").ok().and_then(|s| s.parse().ok()).unwrap_or(60);
        for (label, draw) in [("walls and the meridian plane, half domain", &solved), ("walls and the meridian plane, tiled", &tiled)] {
            rig.frame(draw, false);
            rig.wait();
            let t = Instant::now();
            for _ in 0..frames {
                rig.frame(draw, false);
                rig.wait();
            }
            let ms = t.elapsed().as_secs_f64() * 1e3 / frames as f64;
            eprintln!(
                "flow3 scene, ANF {} cells, {label}, {} x {} px: {ms:.2} ms a frame (submit to done), adapter {}",
                picture.field.cells(),
                size.0,
                size.1,
                adapter.get_info().name
            );
            assert!(ms < 250.0, "{ms} ms a frame");
        }
    }

    /// A meridian cut is the whole plane through the axis, the lee and the windward half, and its edge at the window is straight: from the
    /// side, with the walls hidden, the sheet fills the rows above and below the axis alike, and every row ends at the same column.
    #[test]
    fn a_meridian_cut_is_the_whole_plane_and_ends_in_a_straight_window_edge() {
        let size = (1280u32, 720u32);
        let Some((_, mut rig, bg)) = rig_for(size) else {
            eprintln!("no graphics adapter: flow3 render test skipped");
            return;
        };
        let picture = Arc::new(Prepared3::new(anf()));
        let k = picture.field.layer_of_theta(0.0);
        // a camera that holds the whole window, so the sheet's outline is all in the picture
        let w = picture.window;
        let mut cam = Camera { yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.0, ..Camera::default() };
        cam.fit(glam::Vec3::new(-w.r_max, -w.r_max, w.x.0), glam::Vec3::new(w.r_max, w.r_max, w.x.1), Size::new(size.0 as f32, size.1 as f32));
        let mut view = view_of(&picture, cam);
        view.walls = false;
        view.sheets = picture.sheet_draws(Sheet::Meridian(k), picture.window, false);
        assert_eq!(view.sheets.len(), 2, "the layer and the one across the axis");
        let px = rig.frame(&Draw3 { picture: picture.clone(), view }, true).unwrap();
        let (w_px, h_px) = (size.0 as usize, size.1 as usize);
        let is_lit = |row: usize, x: usize| px[(row * w_px + x) * 4..][..3].iter().zip(bg).any(|(a, b)| (i32::from(*a) - i32::from(b)).abs() > 6);
        let covered = |row: usize| (0..w_px).filter(|&x| is_lit(row, x)).count();
        let (above, below) = ((0..h_px / 2).map(covered).sum::<usize>(), (h_px / 2..h_px).map(covered).sum::<usize>());
        assert!(above > 20_000 && below > 20_000, "both half planes are drawn: {above} above, {below} below");
        assert!((above as f64 / below as f64 - 1.0).abs() < 0.12, "and alike: {above} vs {below}");
        // the window's far edge is a straight line (the plane is turned about the axis by the layer's azimuth, so under perspective the edge
        // leans): the rows beside the body (there is no sheet inside it) end on the line through the first and the last of them, to a pixel
        let mid_row = h_px / 2;
        let rows: Vec<usize> = (mid_row - 250..mid_row - 130).chain(mid_row + 130..mid_row + 250).collect();
        let ends: Vec<(usize, usize)> = rows.iter().filter_map(|&row| Some((row, (0..w_px).rev().find(|&x| is_lit(row, x))?))).collect();
        let ((r0, e0), (r1, e1)) = (ends[0], ends[ends.len() - 1]);
        let worst =
            ends.iter().map(|&(r, e)| (e as f64 - (e0 as f64 + (e1 as f64 - e0 as f64) * (r - r0) as f64 / (r1 - r0) as f64)).abs()).fold(0.0, f64::max);
        assert!(worst <= 1.5, "the sheet ends at a straight edge: {worst} px off the line through ({r0}, {e0}) and ({r1}, {e1})");
    }

    /// The cost of the 3D scene at the sizes the solver will give: the level 0 finner (352 800 cells, `field3_dump --case level0`, if its
    /// file is there) and synthetic bodies of 0.5 and 2 million cells. Prints `FLOW3` lines: building the picture, a sheet, a frame (submit to done).
    /// `cargo nextest run --release -p bowshock flow3_cost --run-ignored only --no-capture`.
    #[test]
    #[ignore = "measures the 3D scene at 0.35, 0.5 and 2 million cells"]
    fn flow3_cost_at_solver_sizes() {
        use super::super::scene3::tests::synthetic;
        let size = (1280u32, 720u32);
        let Some((adapter, mut rig, _)) = rig_for(size) else {
            eprintln!("no graphics adapter: flow3 cost skipped");
            return;
        };
        let mut cases: Vec<(String, Arc<bowshock_core::Field3>)> = Vec::new();
        if let Ok(bytes) = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../out/field3/anf-level0.field3")) {
            cases.push(("anf-level0".into(), Arc::new(super::super::scene3::decode_fixture(&bytes).unwrap())));
        }
        cases.push(("synthetic-0.5M".into(), Arc::new(synthetic(160, 60, 52))));
        cases.push(("synthetic-2M".into(), Arc::new(synthetic(200, 100, 100))));
        let frames = 60usize;
        for (name, field) in cases {
            let cells = field.cells();
            let t0 = Instant::now();
            let picture = Arc::new(Prepared3::new(field.clone()));
            let new_ms = t0.elapsed().as_secs_f64() * 1e3;
            let k = field.layer_of_theta(0.0);
            let t1 = Instant::now();
            let meridian = picture.sheet_draws(Sheet::Meridian(k), picture.window, false);
            let meridian_ms = t1.elapsed().as_secs_f64() * 1e3;
            let t2 = Instant::now();
            let cross = picture.sheet_draws(Sheet::Cross(field.ni / 2), picture.window, false);
            let cross_ms = t2.elapsed().as_secs_f64() * 1e3;
            let view = View3 {
                scalar: Scalar3::Schlieren,
                sheets: [meridian, cross].concat(),
                sheet_range: picture.ranges[Scalar3::Schlieren.index()],
                ..view_of(&picture, camera_for(&picture, size, true, FROM_MINUS_X))
            };
            let draw = Draw3 { picture: picture.clone(), view };
            rig.frame(&draw, false);
            rig.wait();
            let t3 = Instant::now();
            for _ in 0..frames {
                rig.frame(&draw, false);
                rig.wait();
            }
            let frame_ms = t3.elapsed().as_secs_f64() * 1e3 / frames as f64;
            println!(
                "FLOW3 {{\"case\":\"{name}\",\"cells\":{cells},\"walls\":{},\"tiles\":{},\"new_ms\":{new_ms:.1},\"meridian_plane_ms\":{meridian_ms:.1},\"cross_disc_ms\":{cross_ms:.1},\"frame_ms\":{frame_ms:.2},\"adapter\":\"{}\"}}",
                field.walls.len(),
                picture.tiles.len(),
                adapter.get_info().name
            );
        }
    }
}

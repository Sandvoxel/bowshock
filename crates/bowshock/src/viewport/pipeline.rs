//! GPU side of the viewport: a 4x MSAA body pass into an offscreen texture, then a premultiplied-alpha composite onto the
//! target (which already holds the canvas sheet and must never be cleared).

use iced::wgpu::{self, util::DeviceExt};
use iced::widget::shader::{self, Viewport};
use iced::Rectangle;

use super::mesh::Mesh;
use super::solids::GpuVertex;

const SAMPLES: u32 = 4;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    pub view_proj: [[f32; 4]; 4],
    pub light_dir: [f32; 4],
    /// Eye position; `w` is 1 for a perspective view and 0 when orthographic.
    pub eye: [f32; 4],
    pub view_dir: [f32; 4],
    /// View mode (0 solid, 1 cutaway, 2 X-ray, 3 exploded), reveal, explosion 0 to 1.
    pub params: [f32; 4],
    /// Cut wedge: centre direction xy, cosine of half the span, enabled.
    pub cut: [f32; 4],
    /// Logical px per metre (the pipeline multiplies by the device pixel ratio), device pixel ratio.
    pub screen: [f32; 4],
    /// Body, fin, accent, structure, aero, propulsion, recovery, avionics, cut-face, cut-interior, bg-000 (linear).
    pub colors: [[f32; 4]; 11],
}

/// Which parts of the mesh to draw this frame.
#[derive(Debug, Clone, Copy)]
pub struct DrawPlan {
    /// Draw the cutaway cap faces.
    pub caps: bool,
    /// The airframe is translucent (X-ray, auto-reveal): no depth writes.
    pub translucent: bool,
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
    body: wgpu::RenderPipeline,
    translucent: wgpu::RenderPipeline,
    caps: wgpu::RenderPipeline,
    blit: wgpu::RenderPipeline,
    uniforms: wgpu::Buffer,
    uniform_group: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    vertices: Option<(u64, wgpu::Buffer)>,
    ranges: [std::ops::Range<u32>; 3],
    plan: DrawPlan,
    offscreen: Option<Offscreen>,
    /// Widget bounds in physical pixels: x, y, width, height.
    viewport: [f32; 4],
}

impl shader::Pipeline for Pipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("viewport.uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let uniform_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport.uniform_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let uniform_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport.uniform_group"),
            layout: &uniform_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: uniforms.as_entire_binding() }],
        });

        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport.blit_layout"),
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

        let body_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("viewport.body"),
            source: wgpu::ShaderSource::Wgsl(include_str!("body.wgsl").into()),
        });
        let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("viewport.blit"),
            source: wgpu::ShaderSource::Wgsl(include_str!("blit.wgsl").into()),
        });

        let body_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport.body_layout"),
            bind_group_layouts: &[Some(&uniform_layout)],
            immediate_size: 0,
        });
        let blit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport.blit_pipeline_layout"),
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
        // Opaque parts write depth; the translucent airframe (X-ray, auto-reveal) only tests it, so what is behind a pane of
        // shell is still drawn. Back faces are handled in the shader, so nothing is culled.
        let make = |label: &str, depth_write: bool, bias: wgpu::DepthBiasState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&body_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &body_shader,
                    entry_point: Some("vs_main"),
                    buffers: &[Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<GpuVertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32x2, 4 => Float32, 5 => Float32, 6 => Float32, 7 => Float32],
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleList, front_face: wgpu::FrontFace::Ccw, cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(wgpu::CompareFunction::Less),
                    stencil: wgpu::StencilState::default(),
                    bias,
                }),
                multisample: wgpu::MultisampleState { count: SAMPLES, mask: !0, alpha_to_coverage_enabled: false },
                fragment: Some(wgpu::FragmentState {
                    module: &body_shader,
                    entry_point: Some("fs_main"),
                    targets: &[Some(wgpu::ColorTargetState { format, blend: Some(premultiplied), write_mask: wgpu::ColorWrites::ALL })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let body = make("viewport.body_pipeline", true, wgpu::DepthBiasState::default());
        let translucent = make("viewport.translucent_pipeline", false, wgpu::DepthBiasState::default());
        // The cutaway faces lie in the cut planes, where the edges of the cut shells and solids end: a polygon offset towards
        // the eye keeps those coplanar edges from fighting the faces.
        let caps = make("viewport.caps_pipeline", true, wgpu::DepthBiasState { constant: -2, slope_scale: -1.0, clamp: 0.0 });

        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("viewport.blit_pipeline"),
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
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState {
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
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        Self {
            format,
            body,
            translucent,
            caps,
            blit,
            uniforms,
            uniform_group,
            blit_layout,
            vertices: None,
            ranges: [0..0, 0..0, 0..0],
            plan: DrawPlan { caps: false, translucent: false },
            offscreen: None,
            viewport: [0.0; 4],
        }
    }
}

impl Pipeline {
    pub fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        uniforms: &Uniforms,
        plan: DrawPlan,
        mesh: &Mesh,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        let physical = viewport.physical_size();
        let size = (physical.width.max(1), physical.height.max(1));
        if self.offscreen.as_ref().is_none_or(|o| o.size != size) {
            self.offscreen = Some(self.create_offscreen(device, size));
        }

        if mesh.is_empty() {
            // Nothing to draw (and wgpu rejects zero-size buffers).
            self.vertices = None;
        } else if self.vertices.as_ref().is_none_or(|(id, ..)| *id != mesh.id) {
            let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("viewport.vertices"),
                contents: bytemuck::cast_slice(&mesh.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.vertices = Some((mesh.id, buffer));
        }

        let scale = viewport.scale_factor();
        let mut uniforms = *uniforms;
        uniforms.screen = [uniforms.screen[0] * scale, scale, 0.0, 0.0];
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(&uniforms));
        self.ranges = [mesh.inner.clone(), mesh.caps.clone(), mesh.outer.clone()];
        self.plan = plan;
        self.viewport = [bounds.x * scale, bounds.y * scale, bounds.width * scale, bounds.height * scale];
    }

    fn create_offscreen(&self, device: &wgpu::Device, size: (u32, u32)) -> Offscreen {
        let extent = wgpu::Extent3d { width: size.0, height: size.1, depth_or_array_layers: 1 };
        let texture = |label: &str, format, samples, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: extent,
                    mip_level_count: 1,
                    sample_count: samples,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&wgpu::TextureViewDescriptor::default())
        };
        let msaa_view = texture("viewport.msaa", self.format, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let resolve_view = texture("viewport.resolve", self.format, 1, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING);
        let depth_view = texture("viewport.depth", DEPTH_FORMAT, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
        let blit_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport.blit_group"),
            layout: &self.blit_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&resolve_view) }],
        });
        Offscreen { size, msaa_view, resolve_view, depth_view, blit_group }
    }

    pub fn render(&self, encoder: &mut wgpu::CommandEncoder, target: &wgpu::TextureView, clip: &Rectangle<u32>) {
        let (Some(off), Some((_, vertices))) = (&self.offscreen, &self.vertices) else {
            return;
        };
        if clip.width == 0 || clip.height == 0 {
            return;
        }
        // Keep the scissor inside the offscreen texture.
        let sx = clip.x.min(off.size.0.saturating_sub(1));
        let sy = clip.y.min(off.size.1.saturating_sub(1));
        let sw = clip.width.min(off.size.0 - sx).max(1);
        let sh = clip.height.min(off.size.1 - sy).max(1);

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport.body_pass"),
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
            pass.set_bind_group(0, &self.uniform_group, &[]);
            pass.set_vertex_buffer(0, vertices.slice(..));
            // Internal solids, then the cutaway faces, then the airframe (blended over the rest when it is translucent).
            pass.set_pipeline(&self.body);
            pass.draw(self.ranges[0].clone(), 0..1);
            if self.plan.caps {
                pass.set_pipeline(&self.caps);
                pass.draw(self.ranges[1].clone(), 0..1);
            }
            pass.set_pipeline(if self.plan.translucent { &self.translucent } else { &self.body });
            pass.draw(self.ranges[2].clone(), 0..1);
        }

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport.blit_pass"),
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

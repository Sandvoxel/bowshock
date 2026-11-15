//! GPU side of the viewport: a 4x MSAA body pass into an offscreen texture, then a premultiplied-alpha composite onto the
//! target (which already holds the canvas sheet and must never be cleared).

use iced::wgpu::{self, util::DeviceExt};
use iced::widget::shader::{self, Viewport};
use iced::Rectangle;

use super::scene::{Mesh, Vertex};

const SAMPLES: u32 = 4;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    pub view_proj: [[f32; 4]; 4],
    pub light_dir: [f32; 4],
    pub color: [f32; 4],
    pub accent: [f32; 4],
    pub fin: [f32; 4],
    pub view_dir: [f32; 4],
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
    blit: wgpu::RenderPipeline,
    uniforms: wgpu::Buffer,
    uniform_group: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    vertices: Option<(u64, wgpu::Buffer, u32)>,
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

        let body = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("viewport.body_pipeline"),
            layout: Some(&body_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &body_shader,
                entry_point: Some("vs_main"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32, 3 => Float32],
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState { count: SAMPLES, mask: !0, alpha_to_coverage_enabled: false },
            fragment: Some(wgpu::FragmentState {
                module: &body_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState { format, blend: Some(wgpu::BlendState::REPLACE), write_mask: wgpu::ColorWrites::ALL })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

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

        Self { format, body, blit, uniforms, uniform_group, blit_layout, vertices: None, offscreen: None, viewport: [0.0; 4] }
    }
}

impl Pipeline {
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, uniforms: &Uniforms, mesh: &Mesh, bounds: &Rectangle, viewport: &Viewport) {
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
            self.vertices = Some((mesh.id, buffer, mesh.vertices.len() as u32));
        }

        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(uniforms));

        let scale = viewport.scale_factor();
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
        let (Some(off), Some((_, vertices, count))) = (&self.offscreen, &self.vertices) else {
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
            pass.set_pipeline(&self.body);
            pass.set_bind_group(0, &self.uniform_group, &[]);
            pass.set_vertex_buffer(0, vertices.slice(..));
            pass.draw(0..*count, 0..1);
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

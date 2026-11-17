//! The Scene mode of the flow viewer: the 3D picture (the vehicle's wall surface and cut sheets, see [`super::scene3`] and
//! [`super::gpu3`]), what the user chose to show of it ([`SceneState`]), its controls, and the canvas on top that takes the mouse and draws
//! the axes, the free-stream arrow, the readout under the pointer and the colour bars.
//!
//! Navigation follows the kit (design/components/Navigation): right-drag orbits, middle-drag pans, the wheel and a pinch zoom about the
//! cursor, two-finger scroll orbits (pan with `Shift`, zoom with `Cmd`/`Ctrl`), `F` or a double click fits; the left drag orbits too, there
//! being nothing else for it to do here. The camera is the viewport's ([`crate::viewport::camera::Camera`]) in the viewport's world
//! frame (`+Z` along the rocket from the nose, `+Y` up = the lee side).

use super::gpu3::{Draw3, View3};
use super::map;
use super::pick3::{self, Hit, HitKind};
use super::scene3::{world, Extent, Prepared3, Scalar3, Sheet, SheetDraw, WallScalar};
use super::{compact_number, line, put, Msg, DOUBLE_CLICK};
use crate::app::{ui, Ctx};
use crate::theme;
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_2};
use crate::units::Unit;
use crate::viewport::camera::Camera;
use crate::widgets::button::{button as push, Weight};
use crate::widgets::segmented::segmented_fill;
use bowshock_core::Field3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, Path, Stroke};
use iced::widget::Space;
use iced::{alignment, keyboard, mouse, Element, Fill, Length, Point, Rectangle, Renderer, Size, Theme, Vector, Widget};
use std::sync::Arc;
use std::time::Instant;

/// A view direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    /// Three quarters, from ahead of the nose above the lee side.
    Iso,
    /// Side-on: looking along the cross-flow axis, the pitch plane face-on.
    Side,
    /// From above the lee side.
    Top,
    /// Looking at the nose.
    Front,
}

impl Preset {
    /// All four, in the order of the buttons.
    pub const ALL: [Preset; 4] = [Preset::Iso, Preset::Side, Preset::Top, Preset::Front];

    /// The button label.
    pub fn name(self) -> &'static str {
        match self {
            Preset::Iso => "3/4",
            Preset::Side => "Side",
            Preset::Top => "Top",
            Preset::Front => "Nose",
        }
    }

    /// `(yaw, pitch)` of the viewport's camera.
    fn angles(self) -> (f32, f32) {
        use std::f32::consts::{FRAC_PI_2, PI};
        match self {
            Preset::Iso => (-FRAC_PI_2 - 0.7, 0.38),
            Preset::Side => (-FRAC_PI_2, 0.0),
            Preset::Top => (-FRAC_PI_2, FRAC_PI_2 - 0.001),
            Preset::Front => (PI, 0.0),
        }
    }
}

/// The messages of the Scene mode.
#[derive(Debug, Clone)]
pub enum SceneMsg {
    /// Colour the sheets by another quantity.
    Scalar(Scalar3),
    /// Colour the walls by another quantity.
    WallScalar(WallScalar),
    /// Show or hide the meridian sheet.
    Meridian,
    /// The meridian sheet moves by this many cell layers around the axis.
    MeridianStep(i32),
    /// The meridian sheet is at the pitch plane (`false`) or the first fin's plane (`true`).
    MeridianAt(bool),
    /// Show or hide the cross-flow sheet.
    Cross,
    /// The cross-flow sheet is at this station (cell layer along the axis).
    Station(usize),
    /// The cross-flow sheet moves by this many cell layers.
    StationStep(i32),
    /// Draw the meridian sheet in every copy of the solved domain, not as the plane through the axis.
    TileSheets,
    /// Draw the walls in every copy of the solved domain (the whole rocket) or in the solved domain only.
    TileWalls,
    /// Show or hide the wall surface.
    Walls,
    /// How far from the rocket the sheets reach.
    Extent(Extent),
    /// Fit the colour ranges to what is on screen.
    FitRange,
    /// Back to the default colour ranges.
    ResetRange,
    /// Look from this direction.
    View(Preset),
    /// Frame the rocket again.
    Fit,
    /// Orbit by a drag of this many pixels.
    Orbit(Vector),
    /// Pan by a drag of this many pixels.
    Pan(Vector),
    /// Zoom by `factor` keeping the point `offset` px from the centre of the picture where it is.
    Zoom { factor: f32, offset: Vector },
    /// The pointer is at this pixel (`None`: it left).
    Hover(Option<Point>),
    /// The picture's rectangle has this size.
    Resized(Size),
}

/// What is under the pointer.
#[derive(Debug, Clone, Copy)]
pub struct Hover3 {
    /// The pixel in the picture.
    pub at: Point,
    pub hit: Hit,
}

/// The colour ranges the user fitted to the screen.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Fitted {
    /// The sheet scalar and its range.
    pub sheet: Option<(Scalar3, (f32, f32))>,
    /// The wall scalar and its range.
    pub wall: Option<(WallScalar, (f32, f32))>,
}

/// What the Scene mode shows and from where. Session-only.
#[derive(Debug)]
pub struct SceneState {
    pub scalar: Scalar3,
    pub wall_scalar: WallScalar,
    /// The meridian sheet is on, at this cell layer around the axis (`None`: the default, the pitch plane).
    pub meridian: bool,
    layer: Option<usize>,
    /// The cross-flow sheet is on, at this station (`None`: near the fins).
    pub cross: bool,
    station: Option<usize>,
    pub tile_sheets: bool,
    pub tile_walls: bool,
    pub walls: bool,
    pub extent: Extent,
    pub camera: Camera,
    /// The camera is the fit (nobody orbited, panned or zoomed since).
    fitted_view: bool,
    pub size: Size,
    pub fitted: Fitted,
    prepared: Option<Arc<Prepared3>>,
    /// The sheets as drawn, rebuilt when what they depend on changes.
    draws: Vec<SheetDraw>,
    hover: Option<Hover3>,
    /// A snapshot replaced a fitted picture: fit the sheet and wall ranges (the flags) again once the sheets are rebuilt.
    pending_fit: Option<(bool, bool)>,
}

impl Default for SceneState {
    fn default() -> Self {
        SceneState::new()
    }
}

impl SceneState {
    /// A scene with nothing to show: the meridian plane on, the walls, the three-quarter view.
    pub fn new() -> SceneState {
        let (yaw, pitch) = Preset::Iso.angles();
        SceneState {
            scalar: Scalar3::Mach,
            wall_scalar: WallScalar::Cp,
            meridian: true,
            layer: None,
            cross: false,
            station: None,
            tile_sheets: false,
            tile_walls: true,
            walls: true,
            extent: Extent::Near,
            camera: Camera { yaw, pitch, ..Camera::default() },
            fitted_view: true,
            size: Size::ZERO,
            fitted: Fitted::default(),
            prepared: None,
            draws: Vec::new(),
            hover: None,
            pending_fit: None,
        }
    }

    /// The picture on show.
    pub fn picture(&self) -> Option<&Arc<Prepared3>> {
        self.prepared.as_ref()
    }

    /// Shows `field` (the picture the screen wants now), preparing it if it is not the one already on show. The meridian layer and the
    /// station stay where they were while the mesh is the same (the snapshots of a run); a new mesh starts at the defaults.
    pub fn set_field(&mut self, field: Option<&Arc<Field3>>) {
        match (field, &self.prepared) {
            (Some(f), Some(p)) if Arc::ptr_eq(f, &p.field) => {}
            (Some(f), prev) => {
                let same_mesh = prev.as_ref().is_some_and(|p| p.field.mesh_id == f.mesh_id && p.field.nk == f.nk && p.field.ni == f.ni);
                self.prepared = Some(Arc::new(Prepared3::new(f.clone())));
                if !same_mesh {
                    self.layer = None;
                    self.station = None;
                    self.fitted = Fitted::default();
                    self.hover = None;
                    self.refit();
                }
                self.refit_ranges_to_picture();
                self.rebuild();
            }
            (None, Some(_)) => {
                self.prepared = None;
                self.draws.clear();
                self.hover = None;
            }
            (None, None) => {}
        }
    }

    /// The fitted ranges follow a new picture of the same mesh (a snapshot) by being fitted again.
    fn refit_ranges_to_picture(&mut self) {
        if self.fitted == Fitted::default() {
            return;
        }
        // the screen is the same, the values moved: fit again at the next rebuild
        let (sheet, wall) = (self.fitted.sheet.is_some(), self.fitted.wall.is_some());
        self.fitted = Fitted::default();
        self.pending_fit = Some((sheet, wall));
    }

    /// The cell layer of the meridian sheet: the chosen one, else the pitch plane's (azimuth 0).
    pub fn meridian_layer(&self) -> usize {
        match (&self.prepared, self.layer) {
            (Some(p), Some(k)) => k.min(p.field.nk - 1),
            (Some(p), None) => p.field.layer_of_theta(0.0),
            _ => 0,
        }
    }

    /// The station of the cross-flow sheet: the chosen one, else the nearest to 85 % of the way along the rocket.
    pub fn cross_station(&self) -> usize {
        let Some(p) = &self.prepared else { return 0 };
        match self.station {
            Some(i) => i.min(p.field.ni - 1),
            None => {
                let (lo, hi) = p.bounds;
                let want = lo[2] + 0.85 * (hi[2] - lo[2]);
                (0..p.field.ni).min_by(|&a, &b| (p.stations[a] - want).abs().total_cmp(&(p.stations[b] - want).abs())).unwrap_or(0)
            }
        }
    }

    /// Rebuilds the cut sheets from the picture and the choices.
    fn rebuild(&mut self) {
        self.draws.clear();
        let Some(p) = self.prepared.clone() else { return };
        let window = p.window_of(self.extent);
        if self.meridian {
            self.draws.extend(p.sheet_draws(Sheet::Meridian(self.meridian_layer()), window, self.tile_sheets));
        }
        if self.cross {
            self.draws.extend(p.sheet_draws(Sheet::Cross(self.cross_station()), window, false));
        }
        self.hover = None;
        if let Some((sheet, wall)) = self.pending_fit.take() {
            self.fit_ranges(sheet, wall);
        }
    }

    /// The camera framing the vehicle, and the lee half of the sheet window when a sheet is on.
    fn fit_camera(&mut self) {
        let Some(p) = &self.prepared else { return };
        if self.size.width < 2.0 {
            return;
        }
        let (lo, mut hi) = p.bounds;
        if self.meridian || self.cross {
            let w = p.window_of(self.extent);
            hi[1] = hi[1].max(0.6 * w.r_max.min(3.0 * (hi[2] - lo[2])));
        }
        self.camera.fit(glam::Vec3::from(lo), glam::Vec3::from(hi), self.size);
        self.fitted_view = true;
    }

    fn refit(&mut self) {
        if self.fitted_view {
            self.fit_camera();
        }
    }

    /// The colour range of the sheet scalar in use: the fitted one, else the picture's.
    pub fn sheet_range(&self) -> (f32, f32) {
        match (&self.fitted.sheet, &self.prepared) {
            (Some((s, r)), _) if *s == self.scalar => *r,
            (_, Some(p)) => p.ranges[self.scalar.index()],
            _ => (0.0, 1.0),
        }
    }

    /// The colour range of the wall scalar in use.
    pub fn wall_range(&self) -> (f32, f32) {
        match (&self.fitted.wall, &self.prepared) {
            (Some((s, r)), _) if *s == self.wall_scalar => *r,
            (_, Some(p)) => p.wall_ranges[self.wall_scalar.index()],
            _ => (0.0, 1.0),
        }
    }

    /// Whether either colour range is the user's fit.
    pub fn is_fitted(&self) -> bool {
        self.fitted.sheet.is_some_and(|(s, _)| s == self.scalar) || self.fitted.wall.is_some_and(|(s, _)| s == self.wall_scalar)
    }

    /// Fits the colour ranges to what is on screen: robust percentiles of the sheet values and wall values that project inside the picture.
    fn fit_ranges(&mut self, sheet: bool, wall: bool) {
        let Some(p) = self.prepared.clone() else { return };
        let on_screen = |pos: [f32; 3]| {
            let s = self.camera.project(glam::Vec3::from(world(pos)), self.size);
            s.x >= 0.0 && s.y >= 0.0 && s.x <= self.size.width && s.y <= self.size.height
        };
        if sheet && self.sheet_visible() {
            let which = self.scalar.index();
            let mut values: Vec<f32> = Vec::new();
            for d in &self.draws {
                values.extend(d.mesh.vertices.iter().filter(|v| v.values[which].is_finite() && on_screen(v.pos)).map(|v| v.values[which]));
            }
            if let Some(r) = robust_range(self.scalar == Scalar3::Cp, self.scalar == Scalar3::Schlieren, &mut values) {
                self.fitted.sheet = Some((self.scalar, r));
            }
        }
        if wall && self.walls {
            let which = self.wall_scalar.index();
            let m = &p.walls;
            let body = m.triangles[..m.body_indices as usize].iter().copied().max().map_or(0, |i| i as usize + 1);
            let mut values: Vec<f32> = m.vertices[..body].iter().filter(|v| on_screen(v.pos)).map(|v| v.values[which]).filter(|v| v.is_finite()).collect();
            if let Some(r) = robust_range(self.wall_scalar == WallScalar::Cp, false, &mut values) {
                self.fitted.wall = Some((self.wall_scalar, r));
            }
        }
    }

    fn sheet_visible(&self) -> bool {
        !self.draws.is_empty()
    }

    /// Handles a message of the scene.
    pub fn update(&mut self, msg: SceneMsg) {
        match msg {
            SceneMsg::Scalar(s) => {
                self.scalar = s;
                self.fitted.sheet = self.fitted.sheet.filter(|(f, _)| *f == s);
            }
            SceneMsg::WallScalar(s) => {
                self.wall_scalar = s;
                self.fitted.wall = self.fitted.wall.filter(|(f, _)| *f == s);
            }
            SceneMsg::Meridian => {
                self.meridian = !self.meridian;
                self.rebuild();
                self.refit();
            }
            SceneMsg::MeridianStep(by) => {
                let Some(p) = &self.prepared else { return };
                let nk = p.field.nk as i32;
                self.layer = Some(((self.meridian_layer() as i32 + by).rem_euclid(nk)) as usize);
                self.meridian = true;
                self.rebuild();
            }
            SceneMsg::MeridianAt(fin) => {
                let Some(p) = &self.prepared else { return };
                let theta = if fin { p.field.fin_theta.first().copied().unwrap_or(0.0) } else { 0.0 };
                self.layer = Some(p.field.layer_of_theta(theta));
                self.meridian = true;
                self.rebuild();
            }
            SceneMsg::Cross => {
                self.cross = !self.cross;
                self.rebuild();
                self.refit();
            }
            SceneMsg::Station(i) => {
                self.station = Some(i);
                self.cross = true;
                self.rebuild();
            }
            SceneMsg::StationStep(by) => {
                let Some(p) = &self.prepared else { return };
                let at = self.cross_station() as i32 + by;
                self.station = Some(at.clamp(0, p.field.ni as i32 - 1) as usize);
                self.cross = true;
                self.rebuild();
            }
            SceneMsg::TileSheets => {
                self.tile_sheets = !self.tile_sheets;
                self.rebuild();
            }
            SceneMsg::TileWalls => {
                self.tile_walls = !self.tile_walls;
                self.hover = None;
            }
            SceneMsg::Walls => self.walls = !self.walls,
            SceneMsg::Extent(e) => {
                self.extent = e;
                self.rebuild();
                self.refit();
            }
            SceneMsg::FitRange => self.fit_ranges(true, true),
            SceneMsg::ResetRange => self.fitted = Fitted::default(),
            SceneMsg::View(preset) => {
                (self.camera.yaw, self.camera.pitch) = preset.angles();
                self.camera.roll = 0.0;
                self.fit_camera();
            }
            SceneMsg::Fit => self.fit_camera(),
            SceneMsg::Orbit(by) => {
                self.camera.orbit(by);
                self.fitted_view = false;
            }
            SceneMsg::Pan(by) => {
                self.camera.pan(by);
                self.fitted_view = false;
            }
            SceneMsg::Zoom { factor, offset } => {
                self.camera.zoom(factor, offset);
                self.fitted_view = false;
            }
            SceneMsg::Hover(at) => self.hover = at.and_then(|at| self.pick(at)),
            SceneMsg::Resized(size) => {
                self.size = size;
                self.refit();
            }
        }
    }

    /// What the ray through the pixel `at` meets first.
    fn pick(&self, at: Point) -> Option<Hover3> {
        let p = self.prepared.as_ref()?;
        let (o, d) = self.camera.ray(at, self.size);
        let hit = pick3::pick(p, o.to_array(), d.to_array(), &self.draws, p.window_of(self.extent), self.walls, self.tile_walls)?;
        Some(Hover3 { at, hit })
    }

    /// The draw of the current picture and choices, for a picture of `dark` or light theme.
    pub fn draw(&self, dark: bool) -> Option<Draw3> {
        let picture = self.prepared.clone()?;
        let view = View3 {
            camera: self.camera,
            scalar: self.scalar,
            wall_scalar: self.wall_scalar,
            sheets: self.draws.clone(),
            window: picture.window_of(self.extent),
            sheet_range: self.sheet_range(),
            wall_range: self.wall_range(),
            tile_walls: self.tile_walls,
            walls: self.walls,
            dark,
        };
        Some(Draw3 { picture, view })
    }

    /// The lines under the pointer: what the flow or the wall there is, and where.
    pub fn readout(&self, unit: &'static Unit) -> Vec<String> {
        let (Some(h), Some(p)) = (&self.hover, &self.prepared) else { return vec!["hover for values · right-drag orbits · wheel zooms".to_string()] };
        let f = &*p.field;
        let at = h.hit.point;
        let place = format!("x {:.1} {} · r {:.1} {}", unit.from_si(f64::from(at[0])), unit.symbol, unit.from_si(f64::from(at[1].hypot(at[2]))), unit.symbol);
        match h.hit.kind {
            HitKind::Sheet { cell } => {
                let (i, j, k) = (cell % f.ni, cell / f.ni % f.nj, cell / (f.ni * f.nj));
                let v = p.cell_values(i, j, k);
                let rho = f.prim[cell][0];
                let grad = f64::from(f.density_gradient_at(i, j, k)) * unit.to_si;
                let eddy = if f.eddy_ratio.is_empty() { String::new() } else { format!(" · νt/ν {:.0}", v[3]) };
                vec![
                    format!("M {:.2} · Cp {:+.2} · ρ/ρ∞ {rho:.2}{eddy}", v[0], v[1]),
                    format!("{place} · ∇ρ/ρ∞ {} /{} · cell {i}, {j}, {k}", compact_number(grad), unit.symbol),
                ]
            }
            HitKind::Wall { face } => {
                let w = &f.walls[face];
                let cf = w.cf.iter().map(|c| c * c).sum::<f32>().sqrt();
                vec![format!("{} wall · Cp {:+.3} · |Cf| {:.4}", part_name(w.part), w.cp, cf), place]
            }
        }
    }

    /// The palette beside the picture: the mode switch (`mode`, when there is one), what the sheets are coloured by, the sheets, the
    /// walls, the colour ranges and bars, the view.
    pub fn palette<'a>(&'a self, ctx: &'a Ctx, unit: &'static Unit, mode: Option<Element<'a, Msg>>) -> Element<'a, Msg> {
        let Some(p) = &self.prepared else { return Space::new().boxed() };
        let f = &*p.field;
        let toggle =
            |label: &'static str, on: bool, msg: SceneMsg, tip: &'static str| -> Element<'a, Msg> { super::toggle(label, on, Msg::Scene(msg), tip, true) };
        let step = |label: &'static str, msg: SceneMsg, tip: &'static str| -> Element<'a, Msg> {
            push(label, Weight::Secondary).small().on_press(Msg::Scene(msg)).tooltip(tip).into()
        };
        let plain = |label: &'static str, msg: SceneMsg, tip: &'static str| -> Element<'a, Msg> {
            push(label, Weight::Secondary).small().width(Fill).on_press(Msg::Scene(msg)).tooltip(tip).into()
        };
        let sheet_scalars: Vec<(Scalar3, &str)> =
            Scalar3::ALL.iter().copied().filter(|s| *s != Scalar3::Eddy || !f.eddy_ratio.is_empty()).map(|s| (s, s.name())).collect();
        let wall_scalars: Vec<(WallScalar, &str)> = WallScalar::ALL.iter().copied().map(|s| (s, s.name())).collect();
        let theta = f.theta_of_layer(self.meridian_layer()).to_degrees();
        let line = |items: Vec<Element<'a, Msg>>| iced::widget::Row::with_children(items).spacing(SPACE_1).align_y(iced::Center);
        let group = |title: &'static str, items: Vec<Element<'a, Msg>>| {
            let mut all = vec![ui::label(title)];
            all.extend(items);
            iced::widget::Column::with_children(all).spacing(SPACE_1)
        };

        let mut meridian = vec![plain("Pitch plane", SceneMsg::MeridianAt(false), "The plane through the axis on the lee side")];
        if !f.fin_theta.is_empty() {
            meridian.push(plain("Fin plane", SceneMsg::MeridianAt(true), "The plane through a fin"));
        }
        let sheets = group(
            "SHEETS",
            vec![
                segmented_fill(&sheet_scalars, self.scalar, |s| Msg::Scene(SceneMsg::Scalar(s))),
                line(vec![
                    toggle("Meridian", self.meridian, SceneMsg::Meridian, "The whole plane through the axis"),
                    step("◀", SceneMsg::MeridianStep(-1), "Turn the plane one cell layer round the axis"),
                    iced::widget::text(format!("{theta:.0}°")).font(theme::MONO).size(12).style(theme::ink).boxed(),
                    step("▶", SceneMsg::MeridianStep(1), "Turn the plane one cell layer round the axis"),
                ])
                .boxed(),
                line(meridian).boxed(),
                line(vec![
                    toggle("Cross-flow", self.cross, SceneMsg::Cross, "The cut across the rocket at a station"),
                    step("◀", SceneMsg::StationStep(-1), "One cell layer towards the nose"),
                    step("▶", SceneMsg::StationStep(1), "One cell layer towards the tail"),
                ])
                .boxed(),
                canvas::Canvas::new(Station { stations: &p.stations, window: p.window_of(self.extent).x, at: self.cross_station(), on: self.cross })
                    .width(Fill)
                    .height(CONTROL_H)
                    .boxed(),
                line(vec![
                    super::toggle(
                        "Tile sheets",
                        self.tile_sheets,
                        Msg::Scene(SceneMsg::TileSheets),
                        "The meridian layer in every copy of the solved domain, not as the plane",
                        false,
                    ),
                    segmented_fill(&Extent::ALL.map(|e| (e, e.name())), self.extent, |e| Msg::Scene(SceneMsg::Extent(e))),
                ])
                .boxed(),
            ],
        );
        let walls = group(
            "WALLS",
            vec![
                line(vec![
                    toggle("Show", self.walls, SceneMsg::Walls, "The vehicle's surface"),
                    toggle("Whole rocket", self.tile_walls, SceneMsg::TileWalls, "The surface in every copy of the solved domain, not the solved part only"),
                ])
                .boxed(),
                segmented_fill(&wall_scalars, self.wall_scalar, |s| Msg::Scene(SceneMsg::WallScalar(s))),
            ],
        );
        let range = group(
            "COLOUR RANGE",
            vec![line(vec![
                toggle("Fit to screen", self.is_fitted(), SceneMsg::FitRange, "Colour ranges from what is on screen (robust percentiles)"),
                plain("Default", SceneMsg::ResetRange, "The colour ranges of the whole picture"),
            ])
            .boxed()],
        );
        let mut view_buttons: Vec<Element<'a, Msg>> = Preset::ALL.map(|v| plain(v.name(), SceneMsg::View(v), "Look from this direction")).into();
        view_buttons.push(plain("Fit", SceneMsg::Fit, "Frame the rocket again (F)"));
        let view = group("VIEW", vec![line(view_buttons).boxed()]);
        let mut all: Vec<Element<'a, Msg>> = Vec::new();
        all.extend(mode);
        all.extend([sheets.boxed(), walls.boxed(), range.boxed(), view.boxed()]);
        let _ = (ctx, unit);
        iced::widget::scrollable(iced::widget::Column::with_children(all).spacing(SPACE_2).padding(iced::Padding { right: SPACE_2, ..iced::Padding::ZERO }))
            .width(Length::Fixed(PALETTE_W))
            .height(Fill)
            .boxed()
    }

    /// The canvas on top of the picture.
    pub fn overlay(&self, caption: String, unit: &'static Unit, dark: bool) -> Overlay3 {
        let (fs, mach, alpha) = match &self.prepared {
            Some(p) => (p.field.freestream, p.field.mach, p.field.alpha_deg),
            None => ([1.0, 1.0, 0.0, 0.0, 0.7], 1.0, 0.0),
        };
        let mut bars = Vec::new();
        if self.walls {
            bars.push(Bar {
                stops: self.wall_scalar.stops(),
                labels: wall_labels(self.wall_scalar, self.wall_range()),
                mid: self.wall_scalar == WallScalar::Cp,
                title: "wall",
                fitted: self.fitted.wall.is_some_and(|(s, _)| s == self.wall_scalar),
            });
        }
        if !self.draws.is_empty() {
            bars.push(Bar {
                stops: self.scalar.stops(dark),
                labels: scalar_labels(self.scalar, self.sheet_range(), unit),
                mid: self.scalar == Scalar3::Cp,
                title: "sheet",
                fitted: self.fitted.sheet.is_some_and(|(s, _)| s == self.scalar),
            });
        }
        Overlay3 {
            camera: self.camera,
            hover: self.hover.map(|h| h.at),
            readout: self.readout(unit),
            caption,
            free_stream: [fs[2] as f32, fs[3] as f32, fs[1] as f32],
            mach,
            alpha,
            bars,
        }
    }
}

/// The name of a part for the readout.
pub fn part_name(part: bowshock_core::Part) -> String {
    use bowshock_core::Part;
    match part {
        Part::Nose => "Nose".to_string(),
        Part::Body => "Body".to_string(),
        Part::Base => "Base".to_string(),
        Part::Fin(n) => format!("Fin set {}", n + 1),
        Part::Appendage(n) => format!("Appendage {}", n + 1),
        Part::Jet => "Jet".to_string(),
        Part::Other => "Sting".to_string(),
    }
}

/// The range of `values` that leaves out the strongest few per cent at either end: `symmetric` about zero (Cp), `log` (the schlieren's
/// log10: at least a decade wide), else from the low end. `None` without values.
pub(super) fn robust_range(symmetric: bool, log: bool, values: &mut [f32]) -> Option<(f32, f32)> {
    if values.len() < 4 {
        return None;
    }
    values.sort_by(f32::total_cmp);
    let at = |q: f32| values[((values.len() - 1) as f32 * q).round() as usize];
    let (lo, hi) = if log { (at(0.10), at(0.995)) } else { (at(0.02), at(0.98)) };
    let (mut lo, mut hi) = (lo, hi);
    if symmetric {
        let m = (lo.abs().max(hi.abs()) * 20.0).ceil() / 20.0;
        return Some((-m.max(0.05), m.max(0.05)));
    }
    if log && hi - lo < 1.0 {
        lo = hi - 1.0;
    }
    if hi - lo < 1e-3 * (1.0 + hi.abs()) {
        lo -= 0.05;
        hi += 0.05;
    }
    Some((lo, hi))
}

/// The labels at the ends of the sheet colour bar.
fn scalar_labels(s: Scalar3, (lo, hi): (f32, f32), unit: &'static Unit) -> (String, String) {
    match s {
        Scalar3::Mach => (format!("{lo:.2}"), format!("{hi:.2}")),
        Scalar3::Cp => (format!("{lo:+.2}"), format!("{hi:+.2}")),
        Scalar3::Schlieren => {
            let per = |log: f32| compact_number(10f64.powf(f64::from(log)) * unit.to_si);
            (per(lo), format!("{} /{}", per(hi), unit.symbol))
        }
        Scalar3::Eddy => (format!("{lo:.0}"), format!("{hi:.0}")),
    }
}

fn wall_labels(s: WallScalar, (lo, hi): (f32, f32)) -> (String, String) {
    match s {
        WallScalar::Cp => (format!("{lo:+.2}"), format!("{hi:+.2}")),
        WallScalar::Cf => (compact_number(f64::from(lo)), compact_number(f64::from(hi))),
    }
}

/// Width of the palette beside the picture.
const PALETTE_W: f32 = 236.0;

/// A colour bar with its end labels and a title.
struct Bar {
    stops: &'static [[u8; 3]],
    labels: (String, String),
    /// A tick at the middle (Cp: zero).
    mid: bool,
    title: &'static str,
    /// The range is the user's fit, not the default.
    fitted: bool,
}

impl Bar {
    /// Draws the bar on a panel with its top left at `at`, `width` wide.
    fn draw(&self, f: &mut Frame, at: Point, width: f32, t: &crate::tokens::Tokens) {
        use iced::advanced::text::Alignment::{Left, Right};
        let h = 10.0;
        let (x0, x1) = (at.x + 100.0, at.x + width - 72.0);
        let y0 = at.y + 6.0;
        f.fill_rectangle(at, Size::new(width, 22.0), t.bg_100.scale_alpha(0.86));
        let steps = 40;
        for k in 0..steps {
            let (a, b) = (x0 + (x1 - x0) * k as f32 / steps as f32, x0 + (x1 - x0) * (k + 1) as f32 / steps as f32);
            f.fill_rectangle(Point::new(a, y0), Size::new(b - a + 0.5, h), map::color(self.stops, (k as f32 + 0.5) / steps as f32));
        }
        f.stroke(&Path::rectangle(Point::new(x0, y0), Size::new(x1 - x0, h)), line(if self.fitted { t.accent } else { t.line_200 }, 1.0));
        if self.mid {
            let mid = (x0 + x1) / 2.0;
            f.stroke(&Path::line(Point::new(mid, y0 + h), Point::new(mid, y0 + h + 3.0)), line(t.ink_muted, 1.0));
        }
        let cy = at.y + 11.0;
        put(f, self.title, Point::new(at.x + 5.0, cy), t.ink_faint, 11.0, Left, alignment::Vertical::Center);
        put(f, &self.labels.0, Point::new(x0 - 4.0, cy), t.ink_muted, 11.0, Right, alignment::Vertical::Center);
        put(f, &self.labels.1, Point::new(x1 + 4.0, cy), t.ink_muted, 11.0, Left, alignment::Vertical::Center);
    }
}

/// The strip that sets the station of the cross-flow sheet: the axis of the grid from the window's start to its end; a click or a drag
/// along it puts the sheet at the nearest station.
struct Station<'a> {
    stations: &'a [f32],
    window: (f32, f32),
    at: usize,
    on: bool,
}

impl Station<'_> {
    fn x_of(&self, bounds: Size, x: f32) -> f32 {
        let (lo, hi) = self.window;
        6.0 + ((x - lo) / (hi - lo)).clamp(0.0, 1.0) * (bounds.width - 12.0)
    }

    fn nearest(&self, bounds: Size, px: f32) -> usize {
        let (lo, hi) = self.window;
        let x = lo + ((px - 6.0) / (bounds.width - 12.0)).clamp(0.0, 1.0) * (hi - lo);
        (0..self.stations.len()).min_by(|&a, &b| (self.stations[a] - x).abs().total_cmp(&(self.stations[b] - x).abs())).unwrap_or(0)
    }
}

/// Whether the mouse is dragging the strip.
#[derive(Default)]
struct Dragging(bool);

impl canvas::Program<Msg> for Station<'_> {
    type State = Dragging;

    fn update(&self, st: &mut Dragging, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Msg>> {
        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let p = cursor.position_in(bounds)?;
                st.0 = true;
                Some(Action::publish(Msg::Scene(SceneMsg::Station(self.nearest(bounds.size(), p.x)))).and_capture())
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) if st.0 => {
                let p = cursor.position_in(bounds).or_else(|| cursor.position().map(|c| Point::new(c.x - bounds.x, c.y - bounds.y)))?;
                Some(Action::publish(Msg::Scene(SceneMsg::Station(self.nearest(bounds.size(), p.x)))).and_capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => std::mem::take(&mut st.0).then(Action::capture),
            _ => None,
        }
    }

    fn mouse_interaction(&self, _: &Dragging, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::ResizingHorizontally
        } else {
            mouse::Interaction::default()
        }
    }

    fn draw(&self, _: &Dragging, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut f = Frame::new(renderer, bounds.size());
        let mid = bounds.height / 2.0;
        f.stroke(&Path::line(Point::new(6.0, mid), Point::new(bounds.width - 6.0, mid)), line(t.line_200, 1.0));
        // the grid's stations as small ticks (every few of them when they are dense)
        let every = (self.stations.len() / 40).max(1);
        for (i, &x) in self.stations.iter().enumerate().step_by(every) {
            let px = self.x_of(bounds.size(), x);
            if px > 6.0 && px < bounds.width - 6.0 {
                f.stroke(&Path::line(Point::new(px, mid - 2.0), Point::new(px, mid + 2.0)), line(t.line_200, 1.0));
            }
            let _ = i;
        }
        // the nose tip
        let tip = self.x_of(bounds.size(), 0.0);
        f.stroke(&Path::line(Point::new(tip, mid - 6.0), Point::new(tip, mid + 6.0)), line(t.ink_muted, 1.0));
        let x = self.x_of(bounds.size(), self.stations.get(self.at).copied().unwrap_or(0.0));
        let color = if self.on { t.accent } else { t.ink_muted };
        f.stroke(&Path::line(Point::new(x, 3.0), Point::new(x, bounds.height - 3.0)), Stroke::default().with_color(color).with_width(2.0));
        vec![f.into_geometry()]
    }
}

/// The mouse state of the overlay.
#[derive(Default)]
pub struct Interaction3 {
    drag: Option<(Point, Drag)>,
    reported: Size,
    hovering: bool,
    last_click: Option<(Instant, Point)>,
    modifiers: keyboard::Modifiers,
}

#[derive(Clone, Copy, PartialEq)]
enum Drag {
    Orbit,
    Pan,
}

/// The canvas on top of the 3D picture.
pub struct Overlay3 {
    camera: Camera,
    hover: Option<Point>,
    readout: Vec<String>,
    caption: String,
    /// The free-stream direction in the world frame.
    free_stream: [f32; 3],
    mach: f64,
    alpha: f64,
    /// The colour bars in use: the walls', the sheets'.
    bars: Vec<Bar>,
}

impl Overlay3 {
    /// Width and left edge of the colour bars, bottom right.
    fn bars_at(width: f32) -> (f32, f32) {
        let w = 300.0f32.min(width * 0.6);
        (w, width - w - 8.0)
    }

    /// Top left of the free-stream label of width `w`: beside the arrow of the axes (bottom left) when it ends before the bars start, else above the axes, which
    /// the bars (the bottom rows) never reach.
    fn label_at(size: Size, w: f32) -> Point {
        let (origin_x, origin_y) = (40.0, size.height - 40.0);
        let beside = Point::new(origin_x + 96.0 + 44.0, origin_y - 9.0);
        if beside.x + w + 8.0 <= Self::bars_at(size.width).1 {
            beside
        } else {
            Point::new(8.0, origin_y - 60.0)
        }
    }
}

impl canvas::Program<Msg> for Overlay3 {
    type State = Interaction3;

    fn update(&self, st: &mut Interaction3, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Msg>> {
        if st.reported != bounds.size() {
            st.reported = bounds.size();
            return Some(Action::publish(Msg::Scene(SceneMsg::Resized(bounds.size()))));
        }
        let scene = |m: SceneMsg| Some(Action::publish(Msg::Scene(m)).and_capture());
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => {
                st.modifiers = *m;
                None
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key, modifiers, .. }) => {
                let plain = !modifiers.shift() && !modifiers.control() && !modifiers.alt() && !modifiers.logo();
                let is_f = matches!(key, keyboard::Key::Character(c) if c.as_str() == "f");
                (is_f && plain && cursor.position_over(bounds).is_some()).then(|| Action::publish(Msg::Scene(SceneMsg::Fit)).and_capture())
            }
            Event::Mouse(mouse::Event::ButtonPressed(button @ (mouse::Button::Left | mouse::Button::Right | mouse::Button::Middle))) => {
                let p = cursor.position_in(bounds)?;
                if *button == mouse::Button::Left {
                    let now = Instant::now();
                    if st.last_click.is_some_and(|(then, at)| now.duration_since(then) < DOUBLE_CLICK && at.distance(p) < 6.0) {
                        st.last_click = None;
                        return scene(SceneMsg::Fit);
                    }
                    st.last_click = Some((now, p));
                }
                let kind = if *button == mouse::Button::Middle || st.modifiers.shift() { Drag::Pan } else { Drag::Orbit };
                st.drag = Some((p, kind));
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(_)) => st.drag.take().map(|_| Action::capture()),
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let local = cursor.position_in(bounds);
                if let (Some((from, kind)), Some(p)) = (st.drag, local) {
                    st.drag = Some((p, kind));
                    let by = p - from;
                    return scene(if kind == Drag::Pan { SceneMsg::Pan(by) } else { SceneMsg::Orbit(by) });
                }
                match local {
                    Some(p) => {
                        st.hovering = true;
                        Some(Action::publish(Msg::Scene(SceneMsg::Hover(Some(p)))))
                    }
                    None => std::mem::take(&mut st.hovering).then(|| Action::publish(Msg::Scene(SceneMsg::Hover(None)))),
                }
            }
            Event::Mouse(mouse::Event::CursorLeft) => {
                st.drag = None;
                std::mem::take(&mut st.hovering).then(|| Action::publish(Msg::Scene(SceneMsg::Hover(None))))
            }
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let p = cursor.position_in(bounds)?;
                let offset = p - Point::new(bounds.width / 2.0, bounds.height / 2.0);
                match *delta {
                    mouse::ScrollDelta::Lines { y, .. } => scene(SceneMsg::Zoom { factor: 1.2f32.powf(y), offset }),
                    mouse::ScrollDelta::Pixels { x, y } => {
                        if st.modifiers.logo() || st.modifiers.control() {
                            scene(SceneMsg::Zoom { factor: (y * 0.01).exp(), offset })
                        } else if st.modifiers.shift() {
                            scene(SceneMsg::Pan(Vector::new(x, y)))
                        } else {
                            scene(SceneMsg::Orbit(Vector::new(-x, -y)))
                        }
                    }
                }
            }
            Event::Mouse(mouse::Event::PinchGesture { delta, .. }) => {
                let p = cursor.position_in(bounds)?;
                let offset = p - Point::new(bounds.width / 2.0, bounds.height / 2.0);
                scene(SceneMsg::Zoom { factor: crate::widgets::pinch::factor(*delta), offset })
            }
            _ => None,
        }
    }

    fn mouse_interaction(&self, st: &Interaction3, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match st.drag {
            Some((_, Drag::Pan)) => mouse::Interaction::Grabbing,
            Some((_, Drag::Orbit)) => mouse::Interaction::Grabbing,
            None if cursor.is_over(bounds) => mouse::Interaction::Crosshair,
            None => mouse::Interaction::default(),
        }
    }

    fn draw(&self, _: &Interaction3, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        use iced::advanced::text::Alignment::Left;
        let t = theme::tokens(theme);
        let mut f = Frame::new(renderer, bounds.size());
        f.stroke(&Path::rectangle(Point::ORIGIN, bounds.size()), line(t.line_200, 1.0));
        if let Some(at) = self.hover {
            f.stroke(&Path::circle(at, 4.0), line(t.accent, 1.5));
        }
        // the axes of the solver's frame, drawn through the camera: x along the rocket, y across, z up the lee side
        let (right, up) = (self.camera.right(), self.camera.up());
        let screen = |w: glam::Vec3| Vector::new(w.dot(right), -w.dot(up));
        let origin = Point::new(40.0, bounds.height - 40.0);
        for (name, world_dir) in [("x", glam::Vec3::Z), ("y", glam::Vec3::X), ("z", glam::Vec3::Y)] {
            let s = screen(world_dir);
            let to = Point::new(origin.x + 26.0 * s.x, origin.y + 26.0 * s.y);
            f.stroke(&Path::line(origin, to), line(t.ink_muted, 1.4));
            put(
                &mut f,
                name,
                Point::new(origin.x + 33.0 * s.x, origin.y + 33.0 * s.y),
                t.ink_muted,
                11.0,
                iced::advanced::text::Alignment::Center,
                alignment::Vertical::Center,
            );
        }
        // the free stream: where the air comes from (the arrow points the way it goes)
        let fs = glam::Vec3::from(self.free_stream).normalize_or_zero();
        let s = screen(fs);
        let base = Point::new(origin.x + 96.0, origin.y);
        let tip = Point::new(base.x + 34.0 * s.x, base.y + 34.0 * s.y);
        if (s.x * s.x + s.y * s.y).sqrt() > 0.12 {
            f.stroke(&Path::line(Point::new(base.x - 34.0 * s.x, base.y - 34.0 * s.y), tip), line(t.ink, 1.8));
            let ang = s.y.atan2(s.x);
            let head = |da: f32| Point::new(tip.x - 8.0 * (ang + da).cos(), tip.y - 8.0 * (ang + da).sin());
            f.stroke(
                &Path::new(|b| {
                    b.move_to(head(0.45));
                    b.line_to(tip);
                    b.line_to(head(-0.45));
                }),
                line(t.ink, 1.8),
            );
        } else {
            f.stroke(&Path::circle(base, 5.0), line(t.ink, 1.8));
            f.fill(&Path::circle(base, 1.8), t.ink);
        }
        // The label sits beside the arrow when it ends before the colour bars start (bottom right, `bars_at`); in a picture too narrow for both it goes above the
        // axes instead, clear of the bars whatever the size (they take the bottom two or three rows, from the picture's lower edge up).
        let fs_text = format!("V∞ · M {:.2} · α {:.1}°", self.mach, self.alpha);
        let fs_w = 6.6 * fs_text.chars().count() as f32 + 12.0;
        let (bar_w, bars_x) = Self::bars_at(bounds.width);
        let fs_at = Self::label_at(bounds.size(), fs_w);
        f.fill_rectangle(fs_at, Size::new(fs_w, 18.0), t.bg_100.scale_alpha(0.86));
        put(&mut f, &fs_text, Point::new(fs_at.x + 6.0, fs_at.y + 9.0), t.ink, 11.0, Left, alignment::Vertical::Center);

        // the readout under the pointer (a line or two) and the caption: beside the readout when there is room, else under it
        let panel = |f: &mut Frame, s: &str, at: Point, right: bool| {
            let w = 6.6 * s.chars().count() as f32 + 12.0;
            let x = if right { at.x - w } else { at.x };
            f.fill_rectangle(Point::new(x, at.y), Size::new(w, 18.0), t.bg_100.scale_alpha(0.86));
            put(f, s, Point::new(x + 6.0, at.y + 9.0), t.ink, 11.0, Left, alignment::Vertical::Center);
        };
        let widest = self.readout.iter().map(|l| l.chars().count()).max().unwrap_or(0);
        let caption_beside = 6.6 * (self.caption.chars().count() + widest) as f32 + 48.0 < bounds.width && self.readout.len() == 1;
        for (n, l) in self.readout.iter().enumerate() {
            panel(&mut f, l, Point::new(8.0, 8.0 + 19.0 * n as f32), false);
        }
        if !self.caption.is_empty() {
            if caption_beside {
                panel(&mut f, &self.caption, Point::new(bounds.width - 8.0, 8.0), true);
            } else {
                panel(&mut f, &self.caption, Point::new(8.0, 8.0 + 19.0 * self.readout.len() as f32), false);
            }
        }
        // the colour bars, bottom right
        for (n, b) in self.bars.iter().rev().enumerate() {
            b.draw(&mut f, Point::new(bars_x, bounds.height - 24.0 - 24.0 * n as f32), bar_w, t);
        }
        vec![f.into_geometry()]
    }
}

/// The picture of the scene as an element: the GPU draw, the canvas over it.
pub fn picture<'a>(scene: &'a SceneState, ctx: &'a Ctx, caption: String, unit: &'static Unit) -> Option<Element<'a, Msg>> {
    let draw = scene.draw(ctx.dark)?;
    let gpu = iced::widget::shader(draw).width(Fill).height(Fill);
    let overlay = canvas::Canvas::new(scene.overlay(caption, unit, ctx.dark)).width(Fill).height(Fill);
    Some(iced::widget::stack![gpu.boxed(), overlay.boxed()].width(Fill).height(Fill).boxed())
}

#[cfg(test)]
mod tests {
    use super::super::scene3::tests::anf;
    use super::*;

    /// A scene showing the committed finner in a 900 x 600 picture.
    fn scene() -> SceneState {
        let mut s = SceneState::new();
        s.update(SceneMsg::Resized(Size::new(900.0, 600.0)));
        s.set_field(Some(&anf()));
        s
    }

    #[test]
    fn the_free_stream_label_never_collides_with_the_colour_bars_at_any_size() {
        let label_w = 6.6 * "V∞ · M 10.00 · α 10.0°".chars().count() as f32 + 12.0;
        for width in (200..1800).step_by(7) {
            for height in [260.0f32, 400.0, 800.0] {
                let at = Overlay3::label_at(Size::new(width as f32, height), label_w);
                let (_, bars_x) = Overlay3::bars_at(width as f32);
                // two bars of 22 px at rows 24 px apart from the bottom edge: the top of the upper one
                let bars_top = height - 24.0 - 24.0;
                let apart_x = at.x + label_w <= bars_x;
                let apart_y = at.y + 18.0 <= bars_top;
                assert!(apart_x || apart_y, "{width} x {height}: label at {at:?}, bars from x {bars_x}");
                assert!(at.x >= 0.0 && at.y >= 0.0, "{width} x {height}: label at {at:?} is off the picture");
            }
        }
    }

    #[test]
    fn a_new_field_shows_the_meridian_plane_through_the_pitch_plane_and_frames_the_rocket() {
        let s = scene();
        assert_eq!(s.meridian_layer(), 0);
        assert_eq!(s.draws.len(), 2, "the layer and the one across the axis");
        assert!(s.draw(true).is_some());
        // the camera frames the rocket: its corners project into the picture
        let (lo, hi) = s.picture().unwrap().bounds;
        assert!(s.camera.contains(glam::Vec3::from(lo), glam::Vec3::from(hi), s.size));
        // the cross-flow sheet starts near the fins (85 % of the way along) and is off
        let p = s.picture().unwrap();
        assert!(!s.cross && (p.stations[s.cross_station()] - 8.5).abs() < 0.4, "{}", p.stations[s.cross_station()]);
    }

    #[test]
    fn the_meridian_plane_turns_round_the_axis_wraps_and_finds_the_fin() {
        let mut s = scene();
        let nk = s.picture().unwrap().field.nk;
        s.update(SceneMsg::MeridianStep(-1));
        assert_eq!(s.meridian_layer(), nk - 1, "turning back from layer 0 wraps to the last");
        s.update(SceneMsg::MeridianStep(1));
        assert_eq!(s.meridian_layer(), 0);
        s.update(SceneMsg::MeridianAt(true));
        let p = s.picture().unwrap().clone();
        let at = p.field.theta_of_layer(s.meridian_layer());
        assert!((at - p.field.fin_theta[0]).abs() < 0.3, "the fin plane: {at} vs {}", p.field.fin_theta[0]);
        s.update(SceneMsg::MeridianAt(false));
        assert_eq!(s.meridian_layer(), 0);
        // off and on again, and tiled: the layer in every copy, one mesh
        s.update(SceneMsg::Meridian);
        assert!(s.draws.is_empty());
        s.update(SceneMsg::Meridian);
        s.update(SceneMsg::TileSheets);
        assert_eq!(s.draws.len(), 1);
        assert_eq!(s.draws[0].tiles, super::super::scene3::TileSel::All);
    }

    #[test]
    fn the_cross_flow_sheet_goes_to_the_station_asked_for_and_the_extent_widens_the_sheets() {
        let mut s = scene();
        let ni = s.picture().unwrap().field.ni;
        s.update(SceneMsg::Station(10));
        assert!(s.cross && s.cross_station() == 10);
        assert_eq!(s.draws.len(), 3, "the plane's two halves and the disc");
        s.update(SceneMsg::StationStep(-100));
        assert_eq!(s.cross_station(), 0, "clamped at the nose end");
        s.update(SceneMsg::Station(ni + 50));
        assert_eq!(s.cross_station(), ni - 1);
        let triangles = |s: &SceneState| s.draws.iter().map(|d| d.mesh.triangles.len()).sum::<usize>();
        let near = triangles(&s);
        s.update(SceneMsg::Extent(Extent::Wide));
        let wide = triangles(&s);
        s.update(SceneMsg::Extent(Extent::Far));
        let far = triangles(&s);
        assert!(near < wide && wide < far, "{near} < {wide} < {far}");
    }

    #[test]
    fn the_colour_ranges_are_the_pictures_until_the_user_fits_them_to_the_screen() {
        let mut s = scene();
        let default = s.sheet_range();
        assert_eq!(default, s.picture().unwrap().ranges[Scalar3::Mach.index()]);
        assert!(!s.is_fitted());
        s.update(SceneMsg::FitRange);
        assert!(s.is_fitted());
        let fitted = s.sheet_range();
        assert!(fitted != default && fitted.0 < fitted.1, "{fitted:?} vs {default:?}");
        assert!(fitted.0 >= default.0 && fitted.1 <= default.1 + 1e-3, "inside the whole picture's range: {fitted:?} in {default:?}");
        // the wall range is the symmetric Cp range of what is on screen
        let wall = s.wall_range();
        assert_eq!(wall.0, -wall.1);
        // another scalar has no fitted range; back to this one it is gone too (the fit was for Mach), and Default clears it
        s.update(SceneMsg::Scalar(Scalar3::Cp));
        assert_eq!(s.sheet_range(), s.picture().unwrap().ranges[Scalar3::Cp.index()]);
        s.update(SceneMsg::Scalar(Scalar3::Mach));
        assert_eq!(s.sheet_range(), default);
        s.update(SceneMsg::ResetRange);
        assert_eq!(s.wall_range(), s.picture().unwrap().wall_ranges[WallScalar::Cp.index()]);
    }

    #[test]
    fn a_snapshot_of_the_same_mesh_keeps_the_choices_and_refits_the_fitted_ranges() {
        let mut s = scene();
        s.update(SceneMsg::Station(20));
        s.update(SceneMsg::MeridianStep(2));
        s.update(SceneMsg::FitRange);
        let first = s.sheet_range();
        // the snapshot: the same grid, a flow that moved (the Mach number of every cell scaled)
        let mut next = (*anf()).clone();
        next.iteration += 500;
        next.prim.iter_mut().for_each(|q| (q[1], q[2], q[3]) = (q[1] * 0.8, q[2] * 0.8, q[3] * 0.8));
        s.set_field(Some(&Arc::new(next)));
        assert_eq!((s.cross_station(), s.meridian_layer()), (20, 2), "the same mesh: nothing moves");
        assert!(s.is_fitted());
        assert!(s.sheet_range().1 < first.1, "the fit follows the new flow: {:?} vs {first:?}", s.sheet_range());
        // a different mesh starts again at the defaults
        let mut other = (*anf()).clone();
        other.mesh_id ^= 1;
        s.set_field(Some(&Arc::new(other)));
        assert_eq!((s.meridian_layer(), s.is_fitted()), (0, false));
    }

    #[test]
    fn the_pointer_reads_the_sheet_and_the_wall_under_it() {
        let mut s = scene();
        // aim at the middle of the picture, where the camera (framing the rocket) looks
        let centre = Point::new(450.0, 300.0);
        s.update(SceneMsg::Hover(Some(centre)));
        let h = s.hover.expect("the picture is hit at its middle");
        let unit = crate::units::Quantity::Length.unit_for(crate::units::UnitSystem::default());
        let lines = s.readout(unit);
        match h.hit.kind {
            HitKind::Sheet { .. } => assert!(lines[0].starts_with("M ") && lines[1].contains("cell"), "{lines:?}"),
            HitKind::Wall { .. } => assert!(lines[0].contains("wall"), "{lines:?}"),
        }
        s.update(SceneMsg::Hover(None));
        assert!(s.hover.is_none() && s.readout(unit)[0].starts_with("hover for values"));
        // with the sheets and walls off there is nothing to hit
        s.update(SceneMsg::Meridian);
        s.update(SceneMsg::Walls);
        s.update(SceneMsg::Hover(Some(centre)));
        assert!(s.hover.is_none());
    }

    #[test]
    fn navigation_moves_the_camera_and_fit_brings_it_back() {
        let mut s = scene();
        let home = s.camera;
        s.update(SceneMsg::Orbit(Vector::new(40.0, -20.0)));
        s.update(SceneMsg::Zoom { factor: 1.5, offset: Vector::new(30.0, 10.0) });
        s.update(SceneMsg::Pan(Vector::new(15.0, 5.0)));
        assert!(s.camera != home && s.camera.scale > home.scale * 1.4);
        // a resize while the view was moved does not reframe it; a fit does
        let moved = s.camera;
        s.update(SceneMsg::Resized(Size::new(700.0, 500.0)));
        assert_eq!(s.camera, moved);
        s.update(SceneMsg::Fit);
        let (lo, hi) = s.picture().unwrap().bounds;
        assert!(s.camera.contains(glam::Vec3::from(lo), glam::Vec3::from(hi), s.size));
        s.update(SceneMsg::View(Preset::Top));
        assert!(s.camera.pitch > 1.5);
        s.update(SceneMsg::View(Preset::Side));
        assert_eq!(s.camera.pitch, 0.0);
    }

    #[test]
    fn the_robust_range_leaves_out_the_tails_and_is_symmetric_for_cp() {
        let mut v: Vec<f32> = (0..1000).map(|i| i as f32 / 999.0).collect();
        v[0] = -50.0;
        v[999] = 80.0;
        let (lo, hi) = robust_range(false, false, &mut v.clone()).unwrap();
        assert!(lo > 0.0 && lo < 0.05 && hi > 0.95 && hi < 1.0, "{lo} {hi}");
        let (lo, hi) = robust_range(true, false, &mut v.clone()).unwrap();
        assert_eq!(lo, -hi);
        assert!((0.95..=1.05).contains(&hi), "{hi}");
        // the schlieren's log range is at least a decade wide, a flat field a little wider than nothing
        assert!(robust_range(false, true, &mut [2.0; 50]).map(|(l, h)| h - l).unwrap() >= 1.0);
        let (l, h) = robust_range(false, false, &mut [1.0; 50]).unwrap();
        assert!(h > l);
        assert_eq!(robust_range(false, false, &mut [1.0, 2.0]), None);
    }

    #[test]
    fn a_click_on_the_station_strip_picks_the_nearest_station() {
        let stations: Vec<f32> = (0..100).map(|i| i as f32 * 0.1).collect();
        let strip = Station { stations: &stations, window: (0.0, 10.0), at: 0, on: true };
        let size = Size::new(112.0, 28.0);
        assert_eq!(strip.nearest(size, strip.x_of(size, 5.0)), 50);
        assert_eq!(strip.nearest(size, -20.0), 0);
        assert_eq!(strip.nearest(size, 500.0), 99);
    }

    /// What a live snapshot costs the UI thread: [`SceneState::set_field`] with the next snapshot of the same mesh (the picture's derived
    /// data, the sheets on screen, the fitted ranges) at the sizes the solver will give. Prints `SNAPSHOT` lines.
    /// `cargo nextest run --release -p bowshock snapshot_cost --run-ignored only --no-capture`.
    #[test]
    #[ignore = "measures a live snapshot at 0.35, 0.5 and 2 million cells"]
    fn snapshot_cost_at_solver_sizes() {
        use super::super::scene3::tests::synthetic;
        let mut cases: Vec<(String, Arc<bowshock_core::Field3>)> = Vec::new();
        if let Ok(bytes) = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../../out/field3/anf-level0.field3")) {
            cases.push(("anf-level0".into(), Arc::new(super::super::scene3::decode_fixture(&bytes).unwrap())));
        }
        cases.push(("synthetic-0.5M".into(), Arc::new(synthetic(160, 60, 52))));
        cases.push(("synthetic-2M".into(), Arc::new(synthetic(200, 100, 100))));
        for (name, first) in cases {
            let mut s = SceneState::new();
            s.update(SceneMsg::Resized(Size::new(1280.0, 720.0)));
            s.set_field(Some(&first));
            s.update(SceneMsg::Station(first.ni / 2));
            s.update(SceneMsg::FitRange);
            let mut next = (*first).clone();
            next.iteration += 500;
            let t0 = std::time::Instant::now();
            let clone_ms = {
                let c = (*first).clone();
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                drop(c);
                ms
            };
            let next = Arc::new(next);
            let t = std::time::Instant::now();
            s.set_field(Some(&next));
            let set_ms = t.elapsed().as_secs_f64() * 1e3;
            let t = std::time::Instant::now();
            let _ = s.draw(true);
            let draw_ms = t.elapsed().as_secs_f64() * 1e3;
            println!(
                "SNAPSHOT {{\"case\":\"{name}\",\"cells\":{},\"field_clone_ms\":{clone_ms:.1},\"set_field_ms\":{set_ms:.1},\"draw_ms\":{draw_ms:.2}}}",
                first.cells()
            );
        }
    }
}

//! Materials and surface finishes.
//!
//! The densities follow OpenRocket's default material database (Niskanen, *OpenRocket technical documentation*, 2013, and the
//! stock `materials` data file): bulk materials in kg/m^3, surface materials (fabric, film) in kg/m^2 and lines (cord,
//! shroud line) in kg/m. Finish roughness is OpenRocket's `Finish` table.

use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// How a [`Material`]'s density is measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
pub enum MaterialKind {
    /// Solid stock: density in kg/m^3 (tubes, plates, nose cones).
    Bulk,
    /// Sheet stock: density in kg/m^2 (parachute and streamer fabric).
    Surface,
    /// Cord: density in kg/m (shroud lines, shock cord).
    Line,
}

impl MaterialKind {
    /// Unit label of the density: `kg/m³`, `kg/m²` or `kg/m`.
    pub fn density_unit(self) -> &'static str {
        match self {
            MaterialKind::Bulk => "kg/m³",
            MaterialKind::Surface => "kg/m²",
            MaterialKind::Line => "kg/m",
        }
    }
}

/// Stable key of a [`Material`] in the built-in database. Serialised as its key string (`"cardboard"`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MaterialId(&'static str);

macro_rules! material_ids {
    ($($const_name:ident = $key:literal),* $(,)?) => {
        impl MaterialId {
            $(
                #[doc = concat!("The `", $key, "` material.")]
                pub const $const_name: MaterialId = MaterialId($key);
            )*
        }
    };
}

material_ids! {
    CARDBOARD = "cardboard",
    KRAFT_PHENOLIC = "kraft_phenolic",
    BLUE_TUBE = "blue_tube",
    FIBERGLASS = "fiberglass",
    CARBON_FIBER = "carbon_fiber",
    BASSWOOD = "basswood",
    BALSA = "balsa",
    BIRCH_PLYWOOD = "birch_plywood",
    AIRCRAFT_PLYWOOD = "aircraft_plywood",
    ALUMINUM = "aluminum",
    STEEL = "steel",
    BRASS = "brass",
    LEAD = "lead",
    PLA = "pla",
    ABS = "abs",
    POLYCARBONATE = "polycarbonate",
    POLYSTYRENE = "polystyrene",
    EPOXY = "epoxy",
    RIPSTOP_NYLON = "ripstop_nylon",
    MYLAR = "mylar",
    KEVLAR_LINE = "kevlar_line",
    BRAIDED_NYLON_2MM = "braided_nylon_2mm",
    BRAIDED_NYLON_3MM = "braided_nylon_3mm",
    TUBULAR_NYLON_11MM = "tubular_nylon_11mm",
    ELASTIC_CORD_2MM = "elastic_cord_2mm",
    ELASTIC_CORD_6MM = "elastic_cord_6mm",
}

/// One entry of the material database.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Material {
    /// Database key.
    pub id: MaterialId,
    /// Display name.
    pub name: &'static str,
    /// Density: kg/m^3 for [`MaterialKind::Bulk`], kg/m^2 for [`MaterialKind::Surface`], kg/m for [`MaterialKind::Line`].
    pub density: f64,
    /// What the density is per.
    pub kind: MaterialKind,
    /// A typical shell wall thickness for this stock, metres: what a new tube, nose or transition made of it gets. Zero for
    /// surface and line materials.
    pub default_wall: f64,
}

const fn bulk(id: MaterialId, name: &'static str, density: f64, default_wall: f64) -> Material {
    Material { id, name, density, kind: MaterialKind::Bulk, default_wall }
}

const fn surface(id: MaterialId, name: &'static str, density: f64) -> Material {
    Material { id, name, density, kind: MaterialKind::Surface, default_wall: 0.0 }
}

const fn line(id: MaterialId, name: &'static str, density: f64) -> Material {
    Material { id, name, density, kind: MaterialKind::Line, default_wall: 0.0 }
}

/// The built-in database, bulk materials first, then surface, then line.
static MATERIALS: [Material; 26] = [
    bulk(MaterialId::CARDBOARD, "Cardboard", 680.0, 0.0016),
    bulk(MaterialId::KRAFT_PHENOLIC, "Kraft phenolic", 950.0, 0.0016),
    bulk(MaterialId::BLUE_TUBE, "Blue tube", 1300.0, 0.0016),
    bulk(MaterialId::FIBERGLASS, "Fiberglass", 1850.0, 0.0015),
    bulk(MaterialId::CARBON_FIBER, "Carbon fiber", 1780.0, 0.0012),
    bulk(MaterialId::BASSWOOD, "Basswood", 500.0, 0.003),
    bulk(MaterialId::BALSA, "Balsa", 170.0, 0.003),
    bulk(MaterialId::BIRCH_PLYWOOD, "Birch plywood", 630.0, 0.003),
    bulk(MaterialId::AIRCRAFT_PLYWOOD, "Aircraft plywood", 630.0, 0.003),
    bulk(MaterialId::ALUMINUM, "Aluminum", 2700.0, 0.0015),
    bulk(MaterialId::STEEL, "Steel", 7850.0, 0.001),
    bulk(MaterialId::BRASS, "Brass", 8500.0, 0.001),
    bulk(MaterialId::LEAD, "Lead", 11340.0, 0.002),
    bulk(MaterialId::PLA, "PLA", 1250.0, 0.002),
    bulk(MaterialId::ABS, "ABS", 1050.0, 0.002),
    bulk(MaterialId::POLYCARBONATE, "Polycarbonate", 1200.0, 0.002),
    bulk(MaterialId::POLYSTYRENE, "Polystyrene", 1050.0, 0.002),
    bulk(MaterialId::EPOXY, "Epoxy", 1100.0, 0.002),
    surface(MaterialId::RIPSTOP_NYLON, "Ripstop nylon", 0.067),
    surface(MaterialId::MYLAR, "Mylar", 0.021),
    line(MaterialId::KEVLAR_LINE, "Kevlar line (1/8 in)", 0.0035),
    line(MaterialId::BRAIDED_NYLON_2MM, "Braided nylon (2 mm, 1/16 in)", 0.001),
    line(MaterialId::BRAIDED_NYLON_3MM, "Braided nylon (3 mm, 1/8 in)", 0.0035),
    line(MaterialId::TUBULAR_NYLON_11MM, "Tubular nylon (11 mm, 7/16 in)", 0.013),
    line(MaterialId::ELASTIC_CORD_2MM, "Nylon elastic cord (2 mm, 1/16 in)", 0.0018),
    line(MaterialId::ELASTIC_CORD_6MM, "Nylon elastic cord (flat 6 mm, 1/4 in)", 0.0045),
];

/// Every material in the database.
pub fn materials() -> &'static [Material] {
    &MATERIALS
}

/// The materials measured the same way as `kind` (for a material select that only offers what fits).
pub fn materials_of_kind(kind: MaterialKind) -> impl Iterator<Item = &'static Material> {
    MATERIALS.iter().filter(move |m| m.kind == kind)
}

impl MaterialId {
    /// The database key, e.g. `"cardboard"`.
    pub fn key(self) -> &'static str {
        self.0
    }

    /// The material with database key `key`.
    pub fn from_key(key: &str) -> Option<MaterialId> {
        MATERIALS.iter().find(|m| m.id.0 == key).map(|m| m.id)
    }

    /// The database entry.
    pub fn material(self) -> &'static Material {
        // Ids are only made from the table's own keys, so the lookup cannot miss.
        MATERIALS.iter().find(|m| m.id == self).unwrap_or(&MATERIALS[0])
    }

    /// Display name.
    pub fn name(self) -> &'static str {
        self.material().name
    }

    /// Density in the unit of [`Material::kind`].
    pub fn density(self) -> f64 {
        self.material().density
    }

    /// Whether this is bulk, surface or line stock.
    pub fn kind(self) -> MaterialKind {
        self.material().kind
    }

    /// The density if the material is `kind`, else that of `fallback` (a part given the wrong sort of stock still has a mass).
    pub(crate) fn density_as(self, kind: MaterialKind, fallback: MaterialId) -> f64 {
        let m = self.material();
        if m.kind == kind {
            m.density
        } else {
            fallback.material().density
        }
    }
}

impl Serialize for MaterialId {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(self.0)
    }
}

impl<'de> Deserialize<'de> for MaterialId {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let key = String::deserialize(d)?;
        MaterialId::from_key(&key).ok_or_else(|| D::Error::custom(format!("unknown material `{key}`")))
    }
}

/// Surface finish of a part's outside, which sets its skin-friction roughness (OpenRocket's table).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Finish {
    /// 500 µm.
    Rough,
    /// 150 µm.
    Unfinished,
    /// 60 µm (the default).
    #[default]
    RegularPaint,
    /// 20 µm.
    SmoothPaint,
    /// 2 µm.
    Polished,
}

impl Finish {
    /// Every finish, roughest first.
    pub const ALL: [Finish; 5] = [Finish::Rough, Finish::Unfinished, Finish::RegularPaint, Finish::SmoothPaint, Finish::Polished];

    /// Equivalent sand-grain roughness height in metres.
    pub fn roughness(self) -> f64 {
        match self {
            Finish::Rough => 500e-6,
            Finish::Unfinished => 150e-6,
            Finish::RegularPaint => 60e-6,
            Finish::SmoothPaint => 20e-6,
            Finish::Polished => 2e-6,
        }
    }

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            Finish::Rough => "Rough",
            Finish::Unfinished => "Unfinished",
            Finish::RegularPaint => "Regular paint",
            Finish::SmoothPaint => "Smooth paint",
            Finish::Polished => "Polished",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn database_matches_openrocket_defaults() {
        let d = |id: MaterialId| id.density();
        assert_eq!(d(MaterialId::CARDBOARD), 680.0);
        assert_eq!(d(MaterialId::FIBERGLASS), 1850.0);
        assert_eq!(d(MaterialId::CARBON_FIBER), 1780.0);
        assert_eq!(d(MaterialId::BALSA), 170.0);
        assert_eq!(d(MaterialId::ALUMINUM), 2700.0);
        assert_eq!(MaterialId::RIPSTOP_NYLON.kind(), MaterialKind::Surface);
        assert_eq!(d(MaterialId::RIPSTOP_NYLON), 0.067);
        assert_eq!(d(MaterialId::MYLAR), 0.021);
        assert_eq!(MaterialId::BRAIDED_NYLON_3MM.kind(), MaterialKind::Line);
    }

    #[test]
    fn keys_are_unique_and_round_trip() {
        for m in materials() {
            assert_eq!(MaterialId::from_key(m.id.key()), Some(m.id));
            assert_eq!(materials().iter().filter(|o| o.id == m.id).count(), 1);
            assert_eq!(m.kind == MaterialKind::Bulk, m.default_wall > 0.0, "{}", m.name);
        }
        assert_eq!(MaterialId::from_key("unobtainium"), None);
        let json = serde_json::to_string(&MaterialId::MYLAR).unwrap();
        assert_eq!(json, "\"mylar\"");
        assert_eq!(serde_json::from_str::<MaterialId>(&json).unwrap(), MaterialId::MYLAR);
        assert!(serde_json::from_str::<MaterialId>("\"unobtainium\"").is_err());
    }

    #[test]
    fn finishes_are_ordered_by_roughness() {
        let r: Vec<f64> = Finish::ALL.iter().map(|f| f.roughness()).collect();
        assert!(r.windows(2).all(|w| w[0] > w[1]));
        assert_eq!(Finish::default().roughness(), 60e-6);
        assert_eq!(Finish::Polished.name(), "Polished");
    }
}

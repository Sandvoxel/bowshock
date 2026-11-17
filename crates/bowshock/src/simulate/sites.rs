//! Built-in launch sites: picking one sets a flight's site altitude and location (see `left::conditions`).

use bowshock_model::Location;

/// A launch site the Conditions panel offers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaunchSite {
    /// The name in the pick list.
    pub name: &'static str,
    /// Latitude (degrees, north positive).
    pub latitude: f64,
    /// Longitude (degrees, east positive).
    pub longitude: f64,
    /// Elevation above sea level (m).
    pub altitude: f64,
}

/// The sites, in pick-list order.
pub const SITES: [LaunchSite; 3] = [
    // Source: https://www.faroutlaunch.org/about-far (driving coordinates of the FAR launch site); elevation of the Copernicus
    // GLO-90 terrain model at those coordinates, https://api.open-meteo.com/v1/elevation.
    LaunchSite { name: "FAR (Mojave, CA)", latitude: 35.347, longitude: -117.810, altitude: 622.0 },
    // Source: https://orbitcodex.com/facilities/pad/vertical-launch-area (Spaceport America Vertical Launch Area, 1401 m).
    LaunchSite { name: "Spaceport America (NM)", latitude: 32.990, longitude: -106.975, altitude: 1401.0 },
    // Source: https://www.aeropac.org/launch-site-1 (Black Rock launch site, N40 52.241 W119 06.417); elevation of the
    // Copernicus GLO-90 terrain model at those coordinates, https://api.open-meteo.com/v1/elevation.
    LaunchSite { name: "Black Rock Desert (NV)", latitude: 40.871, longitude: -119.107, altitude: 1190.0 },
];

/// The built-in site a flight at `location` with a site altitude of `altitude` (m) is at: the location within 0.01° and the
/// altitude within 5 m of a site's.
pub fn matching(location: Option<Location>, altitude: f64) -> Option<&'static LaunchSite> {
    let at = location?;
    SITES.iter().find(|s| (s.latitude - at.latitude).abs() <= 0.01 && (s.longitude - at.longitude).abs() <= 0.01 && (s.altitude - altitude).abs() <= 5.0)
}

/// The standard-atmosphere temperature at `altitude` above sea level (K): 288.15 K falling 6.5 K per km (ISO 2533).
pub fn isa_temperature(altitude: f64) -> f64 {
    288.15 - 0.0065 * altitude
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_site_matches_its_own_conditions_only() {
        let far = SITES[0];
        let at = Some(Location { latitude: far.latitude, longitude: far.longitude });
        assert_eq!(matching(at, far.altitude), Some(&SITES[0]));
        assert_eq!(matching(at, 100.0), None);
        assert_eq!(matching(None, far.altitude), None);
    }
}

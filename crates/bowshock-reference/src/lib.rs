//! `bowshock-reference`: independent reference solutions the solver is validated against (Taylor-Maccoll cone flow,
//! tangent-cone / shock-expansion theory, axisymmetric potential flow) and the published wind-tunnel data used for total-drag
//! validation ([`arcas`]: NASA TN D-4013/D-4014 sounding-rocket axial force; [`tp2005`]: NASA TP-2005 base pressure with cold air jets).

pub mod anf;
pub mod arcas;
pub mod base_drag;
pub mod blunt_nose;
pub mod fin_wave;
pub mod potential;
pub mod shock_expansion;
pub mod taylor_maccoll;
pub mod tp2005;

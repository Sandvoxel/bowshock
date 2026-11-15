//! Free-text notes on parts (the inspector's `05 NOTES` section). Stored with the document, keyed by part id, so they save,
//! load and undo with everything else.

use crate::rocket::Rocket;
use crate::PartId;

impl Rocket {
    /// The note on part `id` (empty if it has none or `id` is not in the rocket).
    pub fn note(&self, id: PartId) -> &str {
        match self.kind_of(id) {
            Some(_) => self.notes.get(&id).map_or("", String::as_str),
            None => "",
        }
    }

    /// Sets the note on part `id`; an empty note removes it. Returns whether `id` is a part of the rocket.
    pub fn set_note(&mut self, id: PartId, note: &str) -> bool {
        if self.kind_of(id).is_none() {
            return false;
        }
        if note.is_empty() {
            self.notes.remove(&id);
        } else {
            self.notes.insert(id, note.to_string());
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::PartKind;

    #[test]
    fn notes_are_kept_per_part_and_saved() {
        let mut r = Rocket::default();
        let tube = r.insert(0, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        assert_eq!(r.note(tube), "");
        assert!(r.set_note(tube, "Epoxy the coupler"));
        assert_eq!(r.note(tube), "Epoxy the coupler");
        assert!(!r.set_note(999, "nobody home"));
        let again = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(again.note(tube), "Epoxy the coupler");
        assert!(r.set_note(tube, ""));
        assert_eq!(r.note(tube), "");
        // A document with no notes serialises as before: no `notes` key.
        assert!(!r.to_json().contains("notes"));
    }
}

//! Explicit model-label binding; no inference of semantic roles from class numbers.
use crate::{ModelSpec, Result};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadClass {
    ConeRed,
    ConeBlue,
    TrafficLight,
    Crosswalk,
}

pub fn validate(spec: &ModelSpec) -> Result<()> {
    let names = &spec.class_names;
    if (names.is_empty() && spec.class_count != 80)
        || (!names.is_empty()
            && (names.len() != spec.class_count as usize
                || names.iter().any(|n| n.trim().is_empty())
                || names.iter().collect::<BTreeSet<_>>().len() != names.len()))
    {
        return Err("custom class_names must be unique labels in class-ID order".into());
    }
    if spec.road_classes.keys().any(|n| !names.contains(n))
        || spec.road_classes.values().collect::<BTreeSet<_>>().len() != spec.road_classes.len()
    {
        return Err("road_classes requires explicit labels and unique roles".into());
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub struct SemanticMap(pub BTreeMap<u32, RoadClass>);
impl Default for SemanticMap {
    fn default() -> Self {
        // Legacy official COCO baseline only. Custom class tables never inherit it.
        Self(BTreeMap::from([(9, RoadClass::TrafficLight)]))
    }
}
impl SemanticMap {
    pub fn from_spec(spec: &ModelSpec) -> Result<Self> {
        spec.validate()?;
        if spec.class_names.is_empty() {
            return Ok(Self::default());
        }
        Ok(Self(
            spec.class_names
                .iter()
                .enumerate()
                .filter_map(|(i, n)| spec.road_classes.get(n).map(|role| (i as u32, *role)))
                .collect(),
        ))
    }
    pub fn is(&self, id: u32, role: RoadClass) -> bool {
        self.0.get(&id) == Some(&role)
    }
    pub fn has(&self, role: RoadClass) -> bool {
        self.0.values().any(|r| *r == role)
    }
    pub fn lamps_only(&self) -> Self {
        Self(
            self.0
                .iter()
                .filter(|(_, r)| **r == RoadClass::TrafficLight)
                .map(|(i, r)| (*i, *r))
                .collect(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reordered_labels_are_explicit_and_never_inherit_coco() {
        let mut spec: ModelSpec =
            serde_json::from_str(include_str!("../../../config/yolo26-race4.json")).unwrap();
        spec.class_names.swap(0, 2);
        let map = SemanticMap::from_spec(&spec).unwrap();
        assert!(map.is(0, RoadClass::TrafficLight));
        assert!(!map.is(9, RoadClass::TrafficLight));
        spec.road_classes
            .insert("missing".into(), RoadClass::Crosswalk);
        assert!(spec.validate().is_err());
    }
}

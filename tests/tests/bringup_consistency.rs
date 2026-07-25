// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Phase 14.4 — bringup drift gate.
//!
//! `src/sentinel_bringup/launch/system.launch.xml` mirrors the parameter
//! defaults declared in `autoware_sentinel_core::params::declare_parameters`
//! (the `ro!(server, "name", default, ...)` rows). Until the 14.4b
//! declarative node wrappers make the launch file the single source, the two
//! must not drift. This test parses both files textually and compares the
//! full (name, value) set.

use std::collections::BTreeMap;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Extract `ro!(server, "name", default, ...)` pairs from params.rs.
fn params_rs_defaults() -> BTreeMap<String, String> {
    let src = std::fs::read_to_string(
        repo_root().join("src/autoware_sentinel_core/src/params.rs"),
    )
    .expect("read params.rs");

    let mut out = BTreeMap::new();
    let mut rest = src.as_str();
    while let Some(idx) = rest.find("ro!(") {
        rest = &rest[idx + 4..];
        // Fields: server, "name", value, "description"
        let mut fields = rest.splitn(4, ',');
        let _server = fields.next();
        let name = fields
            .next()
            .expect("ro! name field")
            .trim()
            .trim_matches('"')
            .to_string();
        let value = fields
            .next()
            .expect("ro! value field")
            .trim()
            .trim_end_matches("_f64")
            .trim_end_matches("_u64")
            .trim_end_matches("_i64")
            .to_string();
        out.insert(name, value);
    }
    out
}

/// Extract `<param name="..." value="..."/>` rows from the launch XML.
fn launch_xml_params() -> BTreeMap<String, String> {
    let xml = std::fs::read_to_string(
        repo_root().join("src/sentinel_bringup/launch/system.launch.xml"),
    )
    .expect("read system.launch.xml");

    let mut out = BTreeMap::new();
    for line in xml.lines() {
        let line = line.trim();
        if !line.starts_with("<param ") {
            continue;
        }
        let name = line
            .split("name=\"")
            .nth(1)
            .and_then(|s| s.split('"').next())
            .expect("param name attr");
        let value = line
            .split("value=\"")
            .nth(1)
            .and_then(|s| s.split('"').next())
            .expect("param value attr");
        out.insert(name.to_string(), value.to_string());
    }
    out
}

/// Numeric-aware comparison ("0.70" == "0.7", "true" == "true").
fn values_equal(a: &str, b: &str) -> bool {
    if a == b {
        return true;
    }
    match (a.parse::<f64>(), b.parse::<f64>()) {
        (Ok(x), Ok(y)) => (x - y).abs() < 1e-12,
        _ => false,
    }
}

#[test]
fn launch_params_match_declared_defaults() {
    let declared = params_rs_defaults();
    let launched = launch_xml_params();

    assert!(
        !declared.is_empty() && !launched.is_empty(),
        "extraction failed: {} declared / {} launched",
        declared.len(),
        launched.len()
    );

    let mut errors = Vec::new();
    for (name, dv) in &declared {
        match launched.get(name) {
            None => errors.push(format!("missing from launch XML: {name} = {dv}")),
            Some(lv) if !values_equal(dv, lv) => {
                errors.push(format!("value drift: {name} params.rs={dv} launch={lv}"))
            }
            _ => {}
        }
    }
    for name in launched.keys() {
        if !declared.contains_key(name) {
            errors.push(format!("launch XML param not declared in params.rs: {name}"));
        }
    }

    assert!(
        errors.is_empty(),
        "bringup drift ({} issues):\n{}",
        errors.len(),
        errors.join("\n")
    );
}

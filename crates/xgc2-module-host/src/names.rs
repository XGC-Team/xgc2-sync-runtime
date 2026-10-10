//! Identifier grammars shared by the manifest, the control plane and the module loader.

/// Instance, module-handle and channel names: 1..=64 characters, alphanumeric first.
pub fn valid_name(name: &str) -> bool {
    let mut bytes = name.bytes();
    name.len() <= 64
        && bytes.next().is_some_and(|b| b.is_ascii_alphanumeric())
        && bytes.all(|b| b.is_ascii_alphanumeric() || b"._/-".contains(&b))
}

/// Port names as the ABI header defines them: `[a-z0-9_]{1,63}`.
pub fn valid_port_name(name: &str) -> bool {
    (1..=63).contains(&name.len()) && name.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_')
}

/// Entity ids and schema ids: `[A-Za-z0-9._:-]{1,128}`.
pub fn valid_id(id: &str) -> bool {
    (1..=128).contains(&id.len()) && id.bytes().all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b))
}

pub fn valid_sha256(text: &str) -> bool {
    text.len() == 64 && text.bytes().all(|b| b.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grammars() {
        assert!(valid_name("controller") && valid_name("ref.active") && valid_name("a/b-c_d"));
        assert!(!valid_name("") && !valid_name("-x") && !valid_name("a b") && !valid_name(&"x".repeat(65)));
        assert!(valid_port_name("cmd_vel") && valid_port_name("x1"));
        assert!(!valid_port_name("") && !valid_port_name("Cmd") && !valid_port_name("a.b") && !valid_port_name(&"x".repeat(64)));
        assert!(valid_id("xgc2.ugv.unicycle_reference.v1") && valid_id("scout:1"));
        assert!(!valid_id("") && !valid_id("a b"));
        assert!(valid_sha256(&"a".repeat(64)) && !valid_sha256("zz") && !valid_sha256(&"g".repeat(64)));
    }
}

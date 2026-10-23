fn main() {
    // Lets `cargo build` link the extension module on macOS (maturin does this itself).
    pyo3_build_config::add_extension_module_link_args();
}

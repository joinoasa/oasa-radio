fn main() {
    // esp-hal's linker script. It must come last.
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

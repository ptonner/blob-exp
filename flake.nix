{
  description = "Reproducible macroquad development";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        lib = pkgs.lib;
        inherit (pkgs) rust-bin makeRustPlatform;
        rustPlatform = makeRustPlatform {
          cargo = rust-bin.selectLatestNightlyWith (toolchain: toolchain.default);
          rustc = rust-bin.selectLatestNightlyWith (toolchain: toolchain.default);
        };
        nativeBuildInputs = with pkgs; [
          pkg-config
          (rust-bin.stable.latest.default.override { targets = [ "wasm32-unknown-unknown" ]; })
          wasm-bindgen-cli
        ];
        buildInputs = with pkgs; [
          libGL
          udev
          alsa-lib
          vulkan-loader
          mesa
          xorg.libX11
          xorg.libXcursor
          xorg.libXi
          xorg.libXrandr # To use the x11 feature
          libxkbcommon
          wayland # To use the wayland feature
        ];
        webBuildInputs = builtins.concatLists [
          (with pkgs; [
            simple-http-server
            wasm-bindgen-cli
            trunk
          ])
          buildInputs
        ];
        version = "0.1";
        pname = "mquad-flake";
      in
      {
        packages.default = rustPlatform.buildRustPackage {
          inherit
            nativeBuildInputs
            buildInputs
            version
            pname
            ;
          src = ./.;
          cargoLock = {
            lockFile = ./Cargo.lock;
          };
          LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
        };
        packages.web = rustPlatform.buildRustPackage {
          inherit nativeBuildInputs version;
          buildInputs = webBuildInputs;
          pname = pname + "-web";
          src = ./.;
          cargoLock = {
            lockFile = ./Cargo.lock;
          };
          buildPhase = ''
            cargo build --release --target=wasm32-unknown-unknown
            mkdir -p $out
            cp target/wasm32-unknown-unknown/release/${pname}.wasm $out/release.wasm
          '';
          installPhase = "echo 'Skipping installPhase'";
        };
        packages.site =
          let
            web = self.packages.${system}.web;
          in
          pkgs.stdenv.mkDerivation {
            inherit version;
            buildInputs = [
              web
              pkgs.zip
            ];
            pname = pname + "-site";
            src = ./web;
            installPhase = ''
              cp -r . $out
              cp ${web}/* $out
              cd $out
              zip site *
            '';
          };
        devShells.default = pkgs.mkShell rec {
          inherit nativeBuildInputs;
          buildInputs = webBuildInputs;
          LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
        };
      }
    );
}

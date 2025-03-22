{
  description = "Rust development environment with Cargo dependencies";

  inputs = {
    naersk.url = "github:nix-community/naersk/master";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils, naersk }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        naersk-lib = pkgs.callPackage naersk { };

        # Define OS-specific dependencies
        osSpecificDeps = if pkgs.stdenv.isLinux then [
          pkgs.systemd
          pkgs.pkg-config
        ] else if pkgs.stdenv.isDarwin then [
          pkgs.darwin.apple_sdk.frameworks.CoreServices  # Needed for macOS system integration
          pkgs.darwin.apple_sdk.frameworks.CoreServices  # For macOS system integration
          pkgs.darwin.apple_sdk.frameworks.IOKit           # For hardware communication
        ] else [];

      in
      {
        defaultPackage = naersk-lib.buildPackage ./.;
        
        devShell = with pkgs; mkShell {
          buildInputs = [
            rust-analyzer
            cargo
            clippy
            rustc
            rustc
            rustfmt
            pre-commit
            rustPackages.clippy
            rerun
            protobuf
          ] ++ osSpecificDeps;  # Append OS-specific dependencies

          RUST_SRC_PATH = rustPlatform.rustLibSrc;

          shellHook = ''
            export GIT_CONFIG=$PWD/.gitconfig
            export CARGO_NET_GIT_FETCH_WITH_CLI=true
            export GIT_SSH_COMMAND="ssh -F ~/.ssh/config"
            ${if pkgs.stdenv.isLinux then ''
              export PKG_CONFIG_PATH="${pkgs.systemd}/lib/pkgconfig:$PKG_CONFIG_PATH"
            '' else ""}
            
            ${if pkgs.stdenv.isDarwin then ''
              echo "Running on macOS, using Darwin-specific dependencies."
            '' else ""}
            
            echo "Entering Rust development environment..."
            cargo fetch # Pre-fetch dependencies

            # Start Zsh if not already the active shell
            if [ "$SHELL" != "$(command -v zsh)" ]; then
              export SHELL="$(command -v zsh)"
              exec zsh
            fi
          '';
        };
      }
    );
}

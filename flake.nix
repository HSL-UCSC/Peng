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
          systemd  # Required for libudev on Linux
          pkg-config  # Helps with system dependencies
        ] else if pkgs.stdenv.isDarwin then [
          darwin.apple_sdk.frameworks.CoreServices  # Needed for macOS system integration
          darwin.apple_sdk.frameworks.IOKit         # Required for hardware communication
        ] else [];

      in
      {
        defaultPackage = naersk-lib.buildPackage ./.;
        
        devShell = with pkgs; mkShell {
          buildInputs = [
            cargo
            clippy
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

            if [ -z "$ZSH_VERSION" ] && [ -z "$BASH_VERSION" ]; then
              export SHELL="$(which zsh || which bash)"
            fi
          '';
        };
      }
    );
}

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
      in
      {
        defaultPackage = naersk-lib.buildPackage ./.;
        devShell = with pkgs; mkShell {
          buildInputs = [ cargo clippy rustc rustfmt pre-commit rustPackages.clippy rerun protobuf];
          RUST_SRC_PATH = rustPlatform.rustLibSrc;

          shellHook = ''
            export GIT_CONFIG=$PWD/.gitconfig
            export CARGO_NET_GIT_FETCH_WITH_CLI=true
            export LD_LIBRARY_PATH=/Users/m0/Developer/vicon-sys/vendor/libvicon/
            export GIT_SSH_COMMAND="ssh -F ~/.ssh/config"  # Ensure it uses your SSH config
            # Start Zsh if not already the active shell
            echo "Entering Rust development environment..."
            cargo fetch # Pre-fetch dependencies defined in Cargo.toml
            if [ -z "$ZSH_VERSION" ] && [ -z "$BASH_VERSION" ]; then
              export SHELL="$(which zsh || which bash)"
            fi
          '';
        };

      }
    );
}

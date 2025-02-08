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
            if [ "$SHELL" != "$(command -v zsh)" ]; then
              export PATH="$HOME/.cargo/bin:$PATH"
              export SHELL="$(command -v zsh)"
              exec zsh
            fi
          '';
        };

      }
    );
}
# {
#   description = "Rust development environment with Cargo dependencies";
#
#   # Specify the inputs, such as nixpkgs
#   inputs = {
#     nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
#     flake-utils.url = "github:numtide/flake-utils";
#   };
#
#   # Define the outputs
#   outputs = { self, nixpkgs, flake-utils }: flake-utils.lib.eachDefaultSystem (system: let
#     pkgs = import nixpkgs { inherit system; };
#   in {
#     # Define a devShell for development
#     devShell = pkgs.mkShell {
#       # Add Rust and Cargo to the environment
#       buildInputs = [
#         pkgs.rust-analyzer
#         pkgs.protobuf
#         pkgs.rustup
#         pkgs.zsh
#         pkgs.clang
#         pkgs.cmake
#         pkgs.cargo-binstall
#         pkgs.rerun
#       ];
#
#       # Optionally, set environment variables
#       CARGO_HOME = "./.cargo";
#       RUST_BACKTRACE = "1"; # Enable backtrace for debugging
#
#       # Optional shellHook to fetch dependencies when entering the shell
#       shellHook = ''
#         export GIT_CONFIG=$PWD/.gitconfig
#         export CARGO_NET_GIT_FETCH_WITH_CLI=true
#         export LD_LIBRARY_PATH=/Users/m0/Developer/vicon-sys/vendor/libvicon/
#         export GIT_SSH_COMMAND="ssh -F ~/.ssh/config"  # Ensure it uses your SSH config
#         # Start Zsh if not already the active shell
#         echo "Entering Rust development environment..."
#         rustup install stable
#         cargo fetch # Pre-fetch dependencies defined in Cargo.toml
#         if [ "$SHELL" != "$(command -v zsh)" ]; then
#           export PATH="$HOME/.cargo/bin:$PATH"
#           export SHELL="$(command -v zsh)"
#           exec zsh
#         fi
#       '';
#     };
#   });
# }
#

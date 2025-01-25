{
  description = "Rust development environment with Cargo dependencies";

  # Specify the inputs, such as nixpkgs
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  # Define the outputs
  outputs = { self, nixpkgs, flake-utils }: flake-utils.lib.eachDefaultSystem (system: let
    pkgs = import nixpkgs { inherit system; };
  in {
    # Define a devShell for development
    devShell = pkgs.mkShell {
      # Add Rust and Cargo to the environment
      buildInputs = [
        pkgs.rust-analyzer
        pkgs.protobuf
        pkgs.rustup
        pkgs.zsh
        pkgs.clang
        pkgs.cmake
        pkgs.cargo-binstall
      ];

      # Optionally, set environment variables
      CARGO_HOME = "./.cargo";
      RUST_BACKTRACE = "1"; # Enable backtrace for debugging

      # Optional shellHook to fetch dependencies when entering the shell
      shellHook = ''
        export GIT_CONFIG=$PWD/.gitconfig
        export CARGO_NET_GIT_FETCH_WITH_CLI=true
        export LD_LIBRARY_PATH=/Users/m0/Developer/vicon-sys/vendor/libvicon/
        export GIT_SSH_COMMAND="ssh -F ~/.ssh/config"  # Ensure it uses your SSH config
        # export RUSTUP_TOOLCHAIN=nightly

        # Ensure rustup-managed Rust binaries take precedence
        # export PATH="$HOME/.cargo/bin:$PATH"
        # Start Zsh if not already the active shell
        echo "Entering Rust development environment..."
        rustup install stable
        # rustup default nightly
        cargo binstall --force rerun-cli@0.21.0
        cargo fetch # Pre-fetch dependencies defined in Cargo.toml
        if [ "$SHELL" != "$(command -v zsh)" ]; then
          export PATH="$HOME/.cargo/bin:$PATH"
          export SHELL="$(command -v zsh)"
          exec zsh
        fi
      '';
    };
  });
}


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
          pkgs.darwin.apple_sdk.frameworks.CoreServices
          pkgs.darwin.apple_sdk.frameworks.IOKit
        ] else [];
      in
      {
        # Default package build
        defaultPackage = naersk-lib.buildPackage ./.;

        # Default shell - minimal
        devShells.default = with pkgs; mkShell {
          buildInputs = [
            cargo
            clippy
            rustc
            rustfmt
            pre-commit
            rustPackages.clippy
            rerun
            protobuf
            neovim # Include neovim explicitly here
          ] ++ osSpecificDeps;

          shellHook = ''
            echo "Entering default Rust shell..."
          '';
        };

        # Full development shell - zsh based
        devShells.dev = with pkgs; mkShell {
          buildInputs = [
            cargo
            clippy
            rustc
            rustfmt
            pre-commit
            rustPackages.clippy
            rerun
            protobuf
          ] ++ osSpecificDeps;

          RUST_SRC_PATH = pkgs.rustPlatform.rustLibSrc;

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

            echo "Entering full development environment (dev)..."

            # If inside nix shell, make sure to stay in zsh
            if [ -n "$ZSH_VERSION" ]; then
              export SHELL=$(which zsh)
            else
              export SHELL=$(which zsh || echo $SHELL)
            fi

            # Start a login zsh if needed (only if not already inside one)
            if [ -z "$ZSH_VERSION" ]; then
              exec $SHELL -l
            fi
          '';
        };
      }
    );
}

{
  description = "Rust development environment with Cargo dependencies";

  inputs = {
    naersk.url = "github:nix-community/naersk/master";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
    oaapis = {
      url   = "github:HSL-UCSC/Quad-RL";
      flake = false;
    };
  };

  outputs = { self, naersk, fenix, nixpkgs, utils, oaapis }:
    utils.lib.eachDefaultSystem (system: let
    pkgs = import nixpkgs { inherit system; };
    naersk-lib = pkgs.callPackage naersk { };

    fenixToolchain = fenix.packages.${system}.complete.withComponents [
      "rustc"
      "cargo"
      "clippy"
      "rustfmt"
      "rust-src"
    ];

    in rec {
      # This builds your package for running the code.

      packages.default = naersk-lib.buildPackage {
        src = ./.;
        cargo = fenixToolchain;
        rustc = fenixToolchain;
      };

      # Dev shell for just running the code, with a minimal set of Rust tools.
      devShells.run = with pkgs; mkShell {
        buildInputs = [
          fenixToolchain
          pre-commit
          rerun
          protobuf
        ]
        ++ (if stdenv.isLinux then [ pkg-config systemd ] else []);

        shellHook = ''
          export OBSTACLE_AVOIDANCE_APIS=${oaapis.outPath}
          echo "Using ObstacleAvoidanceAPIs from: ${oaapis.outPath}"
        '';
      };

      # Dev shell geared toward development, leaving your system's Neovim in place.
      devShells.dev = with pkgs; mkShell {
        buildInputs = [
          fenixToolchain
          pre-commit
          rerun
          protobuf
        ];
        shellHook = ''
          export GIT_CONFIG=$PWD/.gitconfig
          export CARGO_NET_GIT_FETCH_WITH_CLI=true
          export GIT_SSH_COMMAND="ssh -F ~/.ssh/config"
          export OBSTACLE_AVOIDANCE_APIS=${oaapis.outPath}

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

      # Set the default dev shell to "run"
      devShell = devShells.run;
    });
}

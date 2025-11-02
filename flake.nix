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
    hyrl = {
      url = "git+https://github.com/friend0/ObstacleAvoidanceHyRL?ref=rel-0.1.1";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, naersk, fenix, nixpkgs, utils, oaapis, hyrl }:
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
          hyrl.packages.${system}.default
          netcat
        ]
        ++ (if stdenv.isLinux then [ pkg-config systemd ] else []);

        shellHook = ''
          export OBSTACLE_AVOIDANCE_APIS=${oaapis.outPath}
          echo "Using ObstacleAvoidanceAPIs from: ${oaapis.outPath}"
          echo ""
          echo "HyRL server available: run 'hyrl-server' to start"
        '';
      };

      # Dev shell geared toward development, leaving your system's Neovim in place.
      devShells.dev = with pkgs; mkShell {
        buildInputs = [
          fenixToolchain
          pre-commit
          rerun
          protobuf
          hyrl.packages.${system}.default
          netcat
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
          echo "HyRL server available: run 'hyrl-server' to start"
          cargo fetch # Pre-fetch dependencies

          # Start Zsh if not already the active shell
          if [ "$SHELL" != "$(command -v zsh)" ]; then
            export SHELL="$(command -v zsh)"
            exec zsh
          fi
        '';
      };

      # Dev shell with HyRL server auto-started
      devShells.withServer = with pkgs; mkShell {
        buildInputs = [
          fenixToolchain
          pre-commit
          rerun
          protobuf
          hyrl.packages.${system}.default
          netcat
        ]
        ++ (if stdenv.isLinux then [ pkg-config systemd ] else []);

        shellHook = ''
          export OBSTACLE_AVOIDANCE_APIS=${oaapis.outPath}
          echo "Using ObstacleAvoidanceAPIs from: ${oaapis.outPath}"
          
          echo "Starting HyRL Obstacle Avoidance server in background..."
          # Redirect output to avoid cluttering the shell
          hyrl-server > /tmp/hyrl-server.log 2>&1 &
          SERVER_PID=$!
          
          # Function to cleanup server on exit
          cleanup() {
            echo "Stopping HyRL server (PID: $SERVER_PID)..."
            kill $SERVER_PID 2>/dev/null || true
            wait $SERVER_PID 2>/dev/null || true
            echo "Server stopped."
          }
          trap cleanup EXIT INT TERM
          
          # Wait for server to start
          echo "Waiting for server to start..."
          for i in {1..30}; do
            if nc -z localhost 50051 2>/dev/null; then
              echo "HyRL server ready on localhost:50051"
              echo "Server logs: tail -f /tmp/hyrl-server.log"
              break
            fi
            sleep 1
          done
          
          if ! nc -z localhost 50051 2>/dev/null; then
            echo "Warning: Server may still be starting or failed to start"
            echo "Check logs: cat /tmp/hyrl-server.log"
          fi
          
          echo ""
          echo "Development environment ready with HyRL server running in background"
          echo "Server will stop automatically when you exit this shell"
        '';
      };

      # Set the default dev shell to "run"
      devShell = devShells.run;
    });
}

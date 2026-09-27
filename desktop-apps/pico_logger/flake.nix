{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f system);

    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = import nixpkgs {
            inherit system;
          };

          local-pico-sdk = pkgs.pico-sdk.override {
            withSubmodules = true;
          };
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              cmake
              picotool
              local-pico-sdk
              gcc-arm-embedded
            ];

            shellHook = ''
              export PICO_SDK_PATH=${local-pico-sdk}/lib/pico-sdk
              echo ":3"
            '';
          };
        }
      );
    };
}

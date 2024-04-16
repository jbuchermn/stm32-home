{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };
      in
      {
        devShell =
          with pkgs; mkShell {
            buildInputs = [
              python3

              gcc-arm-embedded
              # pkgsCross.arm-embedded.buildPackages.gdb # see https://github.com/NixOS/nixpkgs/issues/299754

              stlink
              openocd

              bear
              usbutils
              minicom
            ];
          };
      }
    );
}

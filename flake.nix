{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-compat.url = "https://flakehub.com/f/edolstra/flake-compat/1.tar.gz";
  };

  outputs = { nixpkgs, ... }:
    let
      # System types to support.
      supportedSystems = [
        "x86_64-linux"
        "aarch64-darwin"
        "aarch64-linux"
      ];

      forAllSystems = _args:
        nixpkgs.lib.genAttrs
          supportedSystems
          (system:
            _args (import nixpkgs { inherit system; }));
    in
    {
      devShells = forAllSystems
        (pkgs: {
          default = pkgs.mkShell {
            nativeBuildInputs = [
              pkgs.zulu
            ];
          };
        });
    };
}

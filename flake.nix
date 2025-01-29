{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs = { nixpkgs, ... }:
    let
      # System types to support.
      supportedSystems = [
        "x86_64-linux"
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
          default = import ./shell.nix { inherit pkgs; };
        });
    };
}

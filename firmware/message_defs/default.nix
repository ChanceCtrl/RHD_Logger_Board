{
  pkgs,
  flatcc,
}:

# TODO: Finish this

pkgs.stdenv.mkDerivation {
  name = "rhd_fbs";

  src = ./.;

  buildInputs = [
    flatcc
  ];

  buildPhase = ''
    flatc -b --schema can_dbc.fbs
  '';

  installPhase = ''
    mkdir -p $out
    # mv ./can_dbc.fbs $out/dbc.fbs
    # mv ./can_dbc.bfbs $out/dbc.bfbs
  '';
}

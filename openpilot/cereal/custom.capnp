using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct KoalaNavInstruction @0x81c2f05a394cf4af {
  valid @0 :Bool;
  routeId @1 :Text;
  maneuver @2 :Maneuver;
  distanceToManeuver @3 :Float32;  # meters, as reported by navigation provider
  targetLatitude @4 :Float64;
  targetLongitude @5 :Float64;
  targetBearingDeg @6 :Float32;    # road bearing after the maneuver
  sourceMonoTime @7 :UInt64;
  roadName @8 :Text;

  enum Maneuver {
    none @0;
    left @1;
    right @2;
    straight @3;
    uTurn @4;
  }
}

struct KoalaNavRoute @0xaedffd8f31e7b55d {
  routeId @0 :Text;
  revision @1 :UInt32;
  coordinates @2 :List(Coordinate);
  sourceMonoTime @3 :UInt64;

  struct Coordinate {
    latitude @0 :Float64;
    longitude @1 :Float64;
  }
}

struct KoalaNavPlan @0xf35cc4560bbf6ec2 {
  enabled @0 :Bool;
  mode @1 :Mode;
  state @2 :State;
  maneuver @3 :KoalaNavInstruction.Maneuver;
  gpsValid @4 :Bool;
  routeValid @5 :Bool;
  instructionValid @6 :Bool;
  routeMatched @7 :Bool;
  controlAllowed @8 :Bool;  # deliberately false in the shadow-only foundation
  latitude @9 :Float64;
  longitude @10 :Float64;
  horizontalAccuracy @11 :Float32;
  speed @12 :Float32;
  bearingDeg @13 :Float32;
  distanceToManeuver @14 :Float32;
  targetLatitude @15 :Float64;
  targetLongitude @16 :Float64;
  targetBearingDeg @17 :Float32;
  turnAngleDeg @18 :Float32;
  confidence @19 :Float32;
  reason @20 :Text;
  sourceMonoTime @21 :UInt64;
  routeId @22 :Text;

  enum Mode {
    off @0;
    shadow @1;
    simulator @2;
  }

  enum State {
    off @0;
    waitingForGps @1;
    waitingForRoute @2;
    waitingForInstruction @3;
    monitoring @4;
    approach @5;
    ready @6;
    complete @7;
    aborted @8;
  }
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}

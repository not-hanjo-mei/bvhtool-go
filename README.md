# bvhtool-go

BVH whole-body Y-lift + Y-yaw transformer.

## What it does

Edits only the ROOT joint's 6 channels (Xpos Ypos Zpos Zrot Xrot Yrot) of a mocopi-format BVH:

- **Y lift**: adds a constant offset to root Y position (up/down).
- **Y yaw**: rotates the entire character around world Y axis at a pivot point.

Yaw uses proper quaternion composition (`q_yaw * q_current`), not naive Euler addition. Root X/Z positions are rotated around a pivot (start-frame position with Y=0). Continuous Euler unwrapping prevents gimbal-lock jumps. Frame 0 is preserved if it's a neutral T-pose (all root rotations ~0).

## Build

```sh
go build -o bvhtool-go
# Android/Termux static binary:
CGO_ENABLED=0 GOOS=android GOARCH=arm64 go build -o bvhtool-go
```

## Usage

```sh
bvhtool-go -i input.bvh -o output.bvh -y 10 -r 90
```

| Flag | Description |
|------|-------------|
| `-i, --input` | Input BVH (required) |
| `-o, --output` | Output BVH (required) |
| `-y, --lift-cm` | Y lift in cm, +=up -=down |
| `-r, --yaw` | Y yaw in degrees, +=left -=right |

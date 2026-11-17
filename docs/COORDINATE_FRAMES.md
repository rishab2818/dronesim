# Coordinate frames and signs

The simulator uses a right-handed world frame and a right-handed body frame. Both begin aligned.

```text
                       +Z world / body (up)
                              ↑
                              │
                 +Y north / nose forward
                            ↗
                           ●────→ +X east / vehicle right
```

World position and velocity are in metres and metres/second. Body angular velocity is in radians/second. `q_body_to_world` rotates a body vector into world coordinates. The browser maps `(world X,Y,Z)` to Three.js `(X,Z,-Y)` because Three.js is Y-up; this is a basis change, not physics.

With body axes `+X right`, `+Y forward`, `+Z up`, the familiar names need care:

- pitch is rotation about body X. Positive pitch raises the nose; the `W` request is therefore a **negative X moment**.
- roll is rotation about body Y. Positive roll banks the vehicle right.
- yaw is rotation about body Z. Positive yaw turns from +X toward +Y when viewed from above (counter-clockwise in the XY diagram).

Motor order is M1 front-right `(+a,+a)`, M2 rear-right `(+a,-a)`, M3 rear-left `(-a,-a)`, M4 front-left `(-a,+a)`. Spin signs are `+,-,+,-`; `+` means its modeled reaction moment points along body +Z. The signs are asserted in `test_mixer`.


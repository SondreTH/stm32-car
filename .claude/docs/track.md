# The challenge track

First rough map, 2026-11-25, from one walk around the track with a phone
compass. Lengths are rough; the team will walk it again for better numbers.

The car starts at S pointing 353 degrees (nearly north). Bearings are
compass degrees. x = east, y = north, in metres from S.

| Leg | Length | Bearing | Turn before it | Ends at (x, y) |
|---|---|---|---|---|
| 1 | 45 m | 353 | start | (-5.5, 44.7) |
| 2 | 45 m | 84 | 91 right | (39.3, 49.4) |
| 3 | 17 m | 171 | 87 right | (41.9, 32.6) |
| 4 | 30 m | 218 | 47 right | (23.5, 8.9) |
| 5 | 15 m | 264 | 46 right | (8.5, 7.4) |

152 m in all. The loop does not close: after leg 5 the map is about 11 m
north-east of S, so some lengths or bearings are off, or a piece back to S
is missing.

As a route (minus = right):

```
rc
rs 45
rt -91
rs 45
rt -87
rs 17
rt -47
rs 30
rt -46
rs 15
go
```

Still to measure: the exact lengths (legs 3-5 most of all), what comes
after leg 5, where the three 10 s waypoint stops are, obstacles, and the
time limit.
